/* Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Supporting function of UpdateDeviceTree()
 * Function adds memory map entries to the device tree binary
 * dev_tree_add_mem_info() is called at every time when memory type matches
 * conditions */

#include "UpdateDeviceTree.h"
#include "AutoGen.h"
#include <Library/UpdateDeviceTree.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/LocateDeviceTree.h>
#include <Library/BootLinux.h>
#include <Protocol/EFIChipInfoTypes.h>
#include <Protocol/EFIDDRGetConfig.h>
#include <Protocol/EFIRng.h>
#include <Library/PartialGoods.h>

#define NUM_SPLASHMEM_PROP_ELEM 4
#define DEFAULT_CELL_SIZE 2

STATIC struct FstabNode FstabTable = {"/firmware/android/fstab", "dev",
                                      "/soc/"};
STATIC struct FstabNode DynamicFstabTable = {"/firmware/android/fstab",
                                              "status",
                                              ""};
STATIC struct FstabNode VbmetaTable = {"/firmware/android/vbmeta", "parts",
                                      ""};

STATIC struct DisplaySplashBufferInfo splashBuf;
STATIC UINTN splashBufSize = sizeof (splashBuf);

STATIC VOID
PrintSplashMemInfo (CONST CHAR8 *data, INT32 datalen)
{
  UINT32 i, val[NUM_SPLASHMEM_PROP_ELEM] = {0};

  for (i = 0; (i < NUM_SPLASHMEM_PROP_ELEM) && datalen; i++) {
    memcpy (&val[i], data, sizeof (UINT32));
    val[i] = fdt32_to_cpu (val[i]);
    data += sizeof (UINT32);
    datalen -= sizeof (UINT32);
  }

  DEBUG ((EFI_D_VERBOSE, "reg = <0x%08x 0x%08x 0x%08x 0x%08x>\n", val[0],
          val[1], val[2], val[3]));
}

STATIC EFI_STATUS
GetDDRInfo (UINT8 *DdrDeviceType)
{
  EFI_DDRGETINFO_PROTOCOL *DdrInfoIf;
  struct ddr_details_entry_info DdrInfo;
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gEfiDDRGetInfoProtocolGuid, NULL,
                                (VOID **)&DdrInfoIf);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_VERBOSE,
            "INFO: Unable to get DDR Info protocol. DDR type not updated:%r\n",
            Status));
    return Status;
  }

  Status = DdrInfoIf->GetDDRDetails (DdrInfoIf, &DdrInfo);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "INFO: GetDDR details failed\n"));
    return Status;
  }

  *DdrDeviceType = DdrInfo.device_type;
  DEBUG ((EFI_D_VERBOSE, "DDR deviceType:%d", *DdrDeviceType));
  return Status;
}

STATIC EFI_STATUS
GetKaslrSeed (UINT64 *KaslrSeed)
{
  EFI_QCOM_RNG_PROTOCOL *RngIf;
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gQcomRngProtocolGuid, NULL, (VOID **)&RngIf);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_VERBOSE,
            "Error locating PRNG protocol. Fail to generate Kaslr seed:%r\n",
            Status));
    return Status;
  }

  Status = RngIf->GetRNG (RngIf,
                          &gEfiRNGAlgRawGuid,
                          sizeof (UINTN),
                          (UINT8 *)KaslrSeed);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_VERBOSE,
         "Error getting PRNG random number. Fail to generate Kaslr seed:%r\n",
         Status));
    *KaslrSeed = 0;
    return Status;
  }

  return Status;
}

STATIC EFI_STATUS
UpdateSplashMemInfo (VOID *fdt)
{
  EFI_STATUS Status;
  CONST struct fdt_property *Prop = NULL;
  INT32 PropLen = 0;
  INT32 ret = 0;
  UINT32 offset;
  CHAR8 *tmp = NULL;
  UINT32 CONST SplashMemPropSize = NUM_SPLASHMEM_PROP_ELEM * sizeof (UINT32);

  Status =
      gRT->GetVariable ((CHAR16 *)L"DisplaySplashBufferInfo",
                        &gQcomTokenSpaceGuid, NULL, &splashBufSize, &splashBuf);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to get splash buffer info, %r\n", Status));
    goto error;
  }

  DEBUG ((EFI_D_VERBOSE, "Version=%d\nAddr=0x%08x\nSize=0x%08x\n",
          splashBuf.uVersion, splashBuf.uFrameAddr, splashBuf.uFrameSize));

  /* Get offset of the splash memory reservation node */
  ret = fdt_path_offset (fdt, "/reserved-memory/splash_region");
  if (ret < 0) {
    DEBUG ((EFI_D_ERROR, "ERROR: Could not get splash memory region node\n"));
    return EFI_NOT_FOUND;
  }
  offset = ret;
  DEBUG ((EFI_D_VERBOSE, "FB mem node name: %a\n",
          fdt_get_name (fdt, offset, NULL)));

  /* Get the property that specifies the splash memory details */
  Prop = fdt_get_property (fdt, offset, "reg", &PropLen);
  if (!Prop) {
    DEBUG ((EFI_D_ERROR, "ERROR: Could not find the splash reg property\n"));
    return EFI_NOT_FOUND;
  }

  /*
   * The format of the "reg" field is as follows:
   *       <0x0 FBAddress 0x0 FBSize>
   * The expected size of this property is 4 * sizeof(UINT32)
   */
  if (PropLen != SplashMemPropSize) {
    DEBUG (
        (EFI_D_ERROR,
         "ERROR: splash mem reservation node size. Expected: %d, Actual: %d\n",
         SplashMemPropSize, PropLen));
    return EFI_BAD_BUFFER_SIZE;
  }

  DEBUG ((EFI_D_VERBOSE, "Splash memory region before updating:\n"));
  PrintSplashMemInfo (Prop->data, PropLen);

  /* First, update the FBAddress */
  if (CHECK_ADD64 ((UINT64)Prop->data, sizeof (UINT32))) {
    DEBUG ((EFI_D_ERROR, "ERROR: integer Overflow while updating FBAddress"));
    return EFI_BAD_BUFFER_SIZE;
  }
  tmp = (CHAR8 *)Prop->data + sizeof (UINT32);
  splashBuf.uFrameAddr = cpu_to_fdt32 (splashBuf.uFrameAddr);
  memcpy (tmp, &splashBuf.uFrameAddr, sizeof (UINT32));

  /* Next, update the FBSize */
  if (CHECK_ADD64 ((UINT64)tmp, (2 * sizeof (UINT32)))) {
    DEBUG ((EFI_D_ERROR, "ERROR: integer Overflow while updating FBSize"));
    return EFI_BAD_BUFFER_SIZE;
  }
  tmp += (2 * sizeof (UINT32));
  splashBuf.uFrameSize = cpu_to_fdt32 (splashBuf.uFrameSize);
  memcpy (tmp, &splashBuf.uFrameSize, sizeof (UINT32));

  /* Update the property value in place */
  ret = fdt_setprop_inplace (fdt, offset, "reg", Prop->data, PropLen);
  if (ret < 0) {
    DEBUG ((EFI_D_ERROR, "ERROR: Could not update splash mem info\n"));
    return EFI_NO_MAPPING;
  }

  DEBUG ((EFI_D_VERBOSE, "Splash memory region after updating:\n"));
  PrintSplashMemInfo (Prop->data, PropLen);
error:
  return Status;
}

UINT32
fdt_check_header_ext (VOID *fdt)
{
  UINT64 fdt_start, fdt_end;
  UINT32 sum;
  fdt_start = (UINT64)fdt;

  if (fdt_start + fdt_totalsize (fdt) <= fdt_start) {
    return FDT_ERR_BADOFFSET;
  }
  fdt_end = fdt_start + fdt_totalsize (fdt);

  if (!(sum = ADD_OF (fdt_off_dt_struct (fdt), fdt_size_dt_struct (fdt)))) {
    return FDT_ERR_BADOFFSET;
  } else {
    if (CHECK_ADD64 (fdt_start, sum))
      return FDT_ERR_BADOFFSET;
    else if (fdt_start + sum > fdt_end)
      return FDT_ERR_BADOFFSET;
  }
  if (!(sum = ADD_OF (fdt_off_dt_strings (fdt), fdt_size_dt_strings (fdt)))) {
    return FDT_ERR_BADOFFSET;
  } else {
    if (CHECK_ADD64 (fdt_start, sum))
      return FDT_ERR_BADOFFSET;
    else if (fdt_start + sum > fdt_end)
      return FDT_ERR_BADOFFSET;
  }
  if (fdt_start + fdt_off_mem_rsvmap (fdt) > fdt_end)
    return FDT_ERR_BADOFFSET;
  return 0;
}

STATIC
VOID
UpdateGranuleInfo (VOID *fdt)
{
  EFI_STATUS Status = EFI_SUCCESS;
  INT32 GranuleNodeOffset;
  UINT32 GranuleSize;
  INT32 Ret;

  Status = GetGranuleSize (&GranuleSize);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_VERBOSE,
            "Unable to get Granule Size, Status = %r\r\n",
            Status));
    return;
  }

  GranuleNodeOffset = fdt_path_offset (fdt, "/mem-offline");
  if (GranuleNodeOffset < 0) {
    DEBUG ((EFI_D_VERBOSE, "INFO: Could not find mem-offline node.\n"));
    return;
  }

  Ret = fdt_setprop_u32 (fdt, GranuleNodeOffset, "granule", GranuleSize);
  if (Ret) {
    DEBUG ((EFI_D_ERROR, "INFO: Granule size update failed.\n"));
  }
}

STATIC
EFI_STATUS
QueryMemoryCellSize (IN VOID *Fdt, OUT UINT32 *MemoryCellLen)
{
  INT32 RootOffset;
  INT32 PropLen;
  UINT32 AddrCellSize = 0;
  UINT32 SizeCellSize = 0;
  UINT32 *Prop = NULL;

  RootOffset = fdt_path_offset (Fdt, "/");
  if (RootOffset < 0) {
    DEBUG ((EFI_D_ERROR, "Error finding root offset\n"));
    return EFI_NOT_FOUND;
  }

  /* Find address-cells size */
  Prop = (UINT32 *) fdt_getprop (Fdt, RootOffset, "#address-cells", &PropLen);
  if (Prop &&
      PropLen > 0) {
    AddrCellSize = fdt32_to_cpu (*Prop);
  } else {
    DEBUG ((EFI_D_ERROR, "Error finding #address-cells property\n"));
    return EFI_NOT_FOUND;
  }

  /* Find size-cells size */
  Prop =(UINT32 *) fdt_getprop (Fdt, RootOffset, "#size-cells", &PropLen);
  if (Prop &&
      PropLen > 0) {
    SizeCellSize = fdt32_to_cpu (*Prop);
  } else {
    DEBUG ((EFI_D_ERROR, "Error finding #size-cells property\n"));
    return EFI_NOT_FOUND;
  }

  if (AddrCellSize > DEFAULT_CELL_SIZE ||
      SizeCellSize > DEFAULT_CELL_SIZE ||
      SizeCellSize == 0 ||
      AddrCellSize == 0) {
    DEBUG ((EFI_D_ERROR, "Error unsupported cell size value: #address-cell %d" \
              "#size-cell\n", AddrCellSize, SizeCellSize));
    return EFI_INVALID_PARAMETER;
  }

  /* Make sure memory cell size and address cell size are same */
  if (AddrCellSize == SizeCellSize) {
    *MemoryCellLen = AddrCellSize;
  } else {
    DEBUG ((EFI_D_ERROR, "Mismatch memory address cell and size cell size\n"));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
AddMemMap (VOID *Fdt, UINT32 MemNodeOffset, BOOLEAN BootWith32Bit)
{
  EFI_STATUS Status = EFI_NOT_FOUND;
  INT32 ret = 0;
  RamPartitionEntry *RamPartitions = NULL;
  UINT32 NumPartitions = 0;
  UINT32 i = 0;
  UINT32 MemoryCellLen = 0;

  Status = QueryMemoryCellSize (Fdt, &MemoryCellLen);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "ERROR: Not a valid memory node found!\n"));
    return Status;
  }

  Status = GetRamPartitions (&RamPartitions, &NumPartitions);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error returned from GetRamPartitions %r\n", Status));
    return Status;
  }
  if (!RamPartitions) {
    DEBUG ((EFI_D_ERROR, "RamPartitions is NULL\n"));
    return EFI_NOT_FOUND;
  }

  DEBUG ((EFI_D_INFO, "RAM Partitions\r\n"));
  for (i = 0; i < NumPartitions; i++) {
    DEBUG ((EFI_D_INFO, "Add Base: 0x%016lx Available Length: 0x%016lx \n",
            RamPartitions[i].Base, RamPartitions[i].AvailableLength));

    if (MemoryCellLen == 1) {
      ret = dev_tree_add_mem_info (Fdt, MemNodeOffset, RamPartitions[i].Base,
                                    RamPartitions[i].AvailableLength);
    } else {
      ret = dev_tree_add_mem_infoV64 (Fdt, MemNodeOffset,
                                        RamPartitions[i].Base,
                                        RamPartitions[i].AvailableLength);
    }

    if (ret) {
      DEBUG ((EFI_D_ERROR, "Add Base: 0x%016lx Length: 0x%016lx Fail\n",
              RamPartitions[i].Base, RamPartitions[i].AvailableLength));
    }
  }

  FreePool (RamPartitions);
  RamPartitions = NULL;

  return EFI_SUCCESS;
}

/* Supporting function of UpdateDeviceTree()
 * Function first gets the RAM partition table, then passes the pointer to
 * AddMemMap() */
STATIC
EFI_STATUS
target_dev_tree_mem (VOID *fdt, UINT32 MemNodeOffset, BOOLEAN BootWith32Bit)
{
  EFI_STATUS Status;

  /* Get Available memory from partition table */
  Status = AddMemMap (fdt, MemNodeOffset, BootWith32Bit);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR,
            "Invalid memory configuration, check memory partition table: %r\n",
            Status));
    goto out;
  }

  UpdateGranuleInfo (fdt);

out:
  return Status;
}

/* Supporting function of target_dev_tree_mem()
 * Function to add the subsequent RAM partition info to the device tree */
INT32
dev_tree_add_mem_info (VOID *fdt, UINT32 offset, UINT32 addr, UINT32 size)
{
  STATIC INT32 mem_info_cnt = 0;
  INT32 ret = 0;

  if (!mem_info_cnt) {
    /* Replace any other reg prop in the memory node. */
    ret = fdt_setprop_u32 (fdt, offset, "reg", addr);
    mem_info_cnt = 1;
  } else {
    /* Append the mem info to the reg prop for subsequent nodes.  */
    ret = fdt_appendprop_u32 (fdt, offset, "reg", addr);
  }

  if (ret) {
    DEBUG (
        (EFI_D_ERROR, "Failed to add the memory information addr: %d\n", ret));
  }

  ret = fdt_appendprop_u32 (fdt, offset, "reg", size);

  if (ret) {
    DEBUG (
        (EFI_D_ERROR, "Failed to add the memory information size: %d\n", ret));
  }

  return ret;
}

INT32
dev_tree_add_mem_infoV64 (VOID *fdt, UINT32 offset, UINT64 addr, UINT64 size)
{
  STATIC INT32 mem_info_cnt = 0;
  INT32 ret = 0;

  if (!mem_info_cnt) {
    /* Replace any other reg prop in the memory node. */
    ret = fdt_setprop_u64 (fdt, offset, "reg", addr);
    mem_info_cnt = 1;
  } else {
    /* Append the mem info to the reg prop for subsequent nodes.  */
    ret = fdt_appendprop_u64 (fdt, offset, "reg", addr);
  }

  if (ret) {
    DEBUG (
        (EFI_D_ERROR, "Failed to add the memory information addr: %d\n", ret));
  }

  ret = fdt_appendprop_u64 (fdt, offset, "reg", size);

  if (ret) {
    DEBUG (
        (EFI_D_ERROR, "Failed to add the memory information size: %d\n", ret));
  }

  return ret;
}

/* Top level function that updates the device tree. */
EFI_STATUS
UpdateDeviceTree (VOID *fdt,
                  CONST CHAR8 *cmdline,
                  VOID *ramdisk,
                  UINT32 RamDiskSize,
                  BOOLEAN BootWith32Bit)
{
  INT32 ret = 0;
  UINT32 offset;
  UINT32 PaddSize = 0;
  UINT64 KaslrSeed = 0;
  UINT8 DdrDeviceType;
  EFI_STATUS Status;

  /* Check the device tree header */
  ret = fdt_check_header (fdt) || fdt_check_header_ext (fdt);
  if (ret) {
    DEBUG ((EFI_D_ERROR, "ERROR: Invalid device tree header ...\n"));
    return EFI_NOT_FOUND;
  }

  /* Add padding to make space for new nodes and properties. */
  PaddSize = ADD_OF (fdt_totalsize (fdt),
                    DTB_PAD_SIZE + AsciiStrLen (cmdline));
  if (!PaddSize) {
    DEBUG ((EFI_D_ERROR, "ERROR: Integer Overflow: fdt size = %u\n",
            fdt_totalsize (fdt)));
    return EFI_BAD_BUFFER_SIZE;
  }
  ret = fdt_open_into (fdt, fdt, PaddSize);
  if (ret != 0) {
    DEBUG ((EFI_D_ERROR, "ERROR: Failed to move/resize dtb buffer ...\n"));
    return EFI_BAD_BUFFER_SIZE;
  }

  /* Get offset of the memory node */
  ret = fdt_path_offset (fdt, "/memory");
  if (ret < 0) {
    DEBUG ((EFI_D_ERROR, "ERROR: Could not find memory node ...\n"));
    return EFI_NOT_FOUND;
  }

  offset = ret;
  Status = target_dev_tree_mem (fdt, offset, BootWith32Bit);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "ERROR: Cannot update memory node\n"));
    return Status;
  }

  Status = GetDDRInfo (&DdrDeviceType);
  if (Status == EFI_SUCCESS) {
    ret = fdt_appendprop_u32 (fdt, offset, (CONST char *)"ddr_device_type",
                              (UINT32)DdrDeviceType);
    if (ret) {
      DEBUG ((EFI_D_ERROR,
              "ERROR: Cannot update memory node [ddr_device_type] - 0x%x\n",
              ret));
    } else {
      DEBUG ((EFI_D_VERBOSE, "ddr_device_type is added to memory node\n"));
    }
  }

  UpdateSplashMemInfo (fdt);

  /* Get offset of the chosen node */
  ret = fdt_path_offset (fdt, "/chosen");
  if (ret < 0) {
    DEBUG ((EFI_D_ERROR, "ERROR: Could not find chosen node ...\n"));
    return EFI_NOT_FOUND;
  }

  offset = ret;
  if (cmdline) {
    /* Adding the cmdline to the chosen node */
    ret = fdt_appendprop_string (fdt, offset, (CONST char *)"bootargs",
                                 (CONST VOID *)cmdline);
    if (ret) {
      DEBUG ((EFI_D_ERROR,
              "ERROR: Cannot update chosen node [bootargs] - 0x%x\n", ret));
      return EFI_LOAD_ERROR;
    }
  }

  Status = GetKaslrSeed (&KaslrSeed);
  if (Status == EFI_SUCCESS) {
    /* Adding Kaslr Seed to the chosen node */
    ret = fdt_appendprop_u64 (fdt, offset, (CONST char *)"kaslr-seed",
                              (UINT64)KaslrSeed);
    if (ret) {
      DEBUG ((EFI_D_INFO,
              "ERROR: Cannot update chosen node [kaslr-seed] - 0x%x\n", ret));
    } else {
      DEBUG ((EFI_D_INFO, "kaslr-Seed is added to chosen node\n"));
    }
  } else {
    DEBUG ((EFI_D_INFO, "ERROR: Cannot generate Kaslr Seed - %r\n", Status));
  }

  if (RamDiskSize) {
    /* Adding the initrd-start to the chosen node */
    ret = fdt_setprop_u64 (fdt, offset, "linux,initrd-start", (UINT64)ramdisk);
    if (ret) {
      DEBUG ((EFI_D_ERROR,
              "ERROR: Cannot update chosen node [linux,initrd-start] - 0x%x\n",
              ret));
      return EFI_NOT_FOUND;
    }

    /* Adding the initrd-end to the chosen node */
    ret = fdt_setprop_u64 (fdt, offset, "linux,initrd-end",
                           ((UINT64)ramdisk + RamDiskSize));
    if (ret) {
      DEBUG ((EFI_D_ERROR,
              "ERROR: Cannot update chosen node [linux,initrd-end] - 0x%x\n",
              ret));
      return EFI_NOT_FOUND;
    }
  }

  /* Update vbmeta node for PLATFORM_VERSION < 10 */
  if ((ANDROID_PLATFORM_VERSION) &&
      (ANDROID_PLATFORM_VERSION < 10)){
    DEBUG ((EFI_D_ERROR, "Removing the ODM partition from DT.\n"));
    UpdateVbmetaNode (fdt, "odm", NULL);
  }
  /* Update fstab node */
  DEBUG ((EFI_D_VERBOSE, "Start DT fstab node update: %lu ms\n",
          GetTimerCountms ()));
  UpdateFstabNode (fdt);
  DEBUG ((EFI_D_VERBOSE, "End DT fstab node update: %lu ms\n",
          GetTimerCountms ()));

  /* Check partial goods*/
  if (FixedPcdGetBool (EnablePartialGoods)) {
    ret = UpdatePartialGoodsNode (fdt);
    if (ret != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR,
        "Failed to update device tree for partial goods, Status=%r\n",
           ret));
      return ret;
    }
  }
  fdt_pack (fdt);

  return ret;
}

/* Update device tree for vbmeta node */
EFI_STATUS
UpdateVbmetaNode (VOID *fdt,
                  CHAR8* OldPartStr,
                  CHAR8* NewPartStr)
{
  INT32 ParentOffset = 0;
  INT32 NodeOffset = 0;
  CONST struct fdt_property *Prop = NULL;
  INT32 PropLen = 0;
  char *NodeName = NULL;
  EFI_STATUS Status = EFI_SUCCESS;
  CHAR8 *RestParts = NULL;
  CHAR8 *ReplaceStr = NULL;
  CHAR8 *PartitionString = NULL;
  struct FstabNode Table = VbmetaTable;
  struct FstabNode FsTable = FstabTable;
  CHAR8 TempPartition[MAX_PARTITION_NAME_LEN];

  PartitionString = AllocateZeroPool (sizeof (CHAR8) * MAX_PARTITION_NAME_LEN * MAX_NUM_PARTITIONS);
  if (PartitionString == NULL) {
    DEBUG ((EFI_D_ERROR, "Boot device buffer: Out of resources\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto error;
  }

  /* Find the parent node */
  ParentOffset = fdt_path_offset (fdt, Table.ParentNode);
  if (ParentOffset < 0) {
    DEBUG ((EFI_D_ERROR, "Failed to Get parent node: fstab\terror: %d\n",
            ParentOffset));
    Status = EFI_NOT_FOUND;
    goto error;
  }
  DEBUG ((EFI_D_INFO, "Node: %a found.\n",
          fdt_get_name (fdt, ParentOffset, NULL)));

  Prop = fdt_get_property (fdt, ParentOffset, Table.Property, &PropLen);
  NodeName = (char *)(uintptr_t)fdt_get_name (fdt, ParentOffset, NULL);
  if (!Prop) {
    DEBUG ((EFI_D_ERROR, "Property:%a is not found for sub-node:%a\n",
            Table.Property, NodeName));
    Status = EFI_NOT_FOUND;
    goto error;
  } else {
    /*Populate the PartitionString with Prop->data*/
    gBS->CopyMem(PartitionString,(CHAR8 *)Prop->data,AsciiStrLen(Prop->data));

    RestParts = (CHAR8 *)PartitionString;
    ReplaceStr = AsciiStrStr (RestParts, OldPartStr);
    if (!ReplaceStr){
      DEBUG ((EFI_D_ERROR, "Unable to find the %a partition\n",OldPartStr));
      Status = EFI_NOT_FOUND;
      goto error;
    }

    RestParts = AsciiStrStr (ReplaceStr, ",");
    if (RestParts){
      RestParts++;
      gBS->CopyMem (ReplaceStr, RestParts, AsciiStrLen(RestParts));
      ReplaceStr = ReplaceStr + AsciiStrLen(RestParts);
    }

    if (NewPartStr){
      if (RestParts)
        AsciiSPrint(TempPartition, MAX_PARTITION_NAME_LEN, ",%a",NewPartStr);
      else
        AsciiSPrint(TempPartition, MAX_PARTITION_NAME_LEN, "%a",NewPartStr);

      gBS->CopyMem(ReplaceStr, TempPartition, AsciiStrLen(TempPartition));
      ReplaceStr = ReplaceStr+AsciiStrLen(TempPartition);
    }
    /*This case is for extra ','*/
    if (!NewPartStr && !RestParts)
      ReplaceStr = ReplaceStr-1;

    *ReplaceStr = '\0';

    Status = fdt_setprop_string(fdt,
		         ParentOffset,
			 Table.Property,
			 (CHAR8 *)PartitionString);

    if(Status){
      DEBUG ((EFI_D_ERROR, "Unable to set property: %a\n",Table.Property));
      Status = EFI_NOT_FOUND;
      goto error;
    }
    Prop = fdt_get_property (fdt, ParentOffset, Table.Property, &PropLen);
    NodeName = (char *)(uintptr_t)fdt_get_name (fdt, ParentOffset, NULL);

    DEBUG ((EFI_D_VERBOSE, "Property:%a found for sub-node:%a\tProperty:%a\n",Table.Property, NodeName, Prop->data));

    AsciiSPrint(TempPartition, MAX_PARTITION_NAME_LEN, "%a/%a",FsTable.ParentNode,OldPartStr);
    /* Get the Odm Node Offset */
    NodeOffset = fdt_path_offset (fdt, TempPartition);
    if(NodeOffset < 0){
      DEBUG ((EFI_D_ERROR, "Failed to Get parent node: %a\terror: %d\n",
              OldPartStr, ParentOffset));
      Status = EFI_NOT_FOUND;
      goto error;
    }
    /*Remove the Partition Node */
    Status = fdt_del_node(fdt, NodeOffset);
    if(Status){
      DEBUG ((EFI_D_ERROR, "fdt_del_node(OldPartStr): %s\n",fdt_strerror(Status)));
      goto error;
    }
  }
error:
  if (PartitionString){
    FreePool (PartitionString);
    PartitionString = NULL;
  }
  return Status;
}

/* Update device tree for fstab node */
EFI_STATUS
UpdateFstabNode (VOID *fdt)
{
  INT32 ParentOffset = 0;
  INT32 SubNodeOffset = 0;
  CONST struct fdt_property *Prop = NULL;
  INT32 PropLen = 0;
  char *NodeName = NULL;
  EFI_STATUS Status = EFI_SUCCESS;
  CHAR8 *BootDevBuf = NULL;
  CHAR8 *ReplaceStr = NULL;
  CHAR8 *NextStr = NULL;
  struct FstabNode Table = IsDynamicPartitionSupport () ? DynamicFstabTable
                                                         : FstabTable;
  UINT32 DevNodeBootDevLen = 0;
  UINT32 Index = 0;
  UINT32 PaddingEnd = 0;

  /* Find the parent node */
  ParentOffset = fdt_path_offset (fdt, Table.ParentNode);
  if (ParentOffset < 0) {
    DEBUG ((EFI_D_VERBOSE, "Failed to Get parent node: fstab\terror: %d\n",
            ParentOffset));
    return EFI_NOT_FOUND;
  }
  DEBUG ((EFI_D_VERBOSE, "Node: %a found.\n",
          fdt_get_name (fdt, ParentOffset, NULL)));

  if (!IsDynamicPartitionSupport ()) {
    /* Get boot device type */
    BootDevBuf = AllocateZeroPool (sizeof (CHAR8) * BOOT_DEV_MAX_LEN);
    if (BootDevBuf == NULL) {
     DEBUG ((EFI_D_ERROR, "Boot device buffer: Out of resources\n"));
     return EFI_OUT_OF_RESOURCES;
    }

    Status = GetBootDevice (BootDevBuf, BOOT_DEV_MAX_LEN);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Failed to get Boot Device: %r\n", Status));
      FreePool (BootDevBuf);
      BootDevBuf = NULL;
      return Status;
    }
  }

  /* Get properties of all sub nodes */
  for (SubNodeOffset = fdt_first_subnode (fdt, ParentOffset);
       SubNodeOffset >= 0;
       SubNodeOffset = fdt_next_subnode (fdt, SubNodeOffset)) {
    Prop = fdt_get_property (fdt, SubNodeOffset, Table.Property, &PropLen);
    NodeName = (char *)(uintptr_t)fdt_get_name (fdt, SubNodeOffset, NULL);
    if (!Prop) {
      DEBUG ((EFI_D_VERBOSE, "Property:%a is not found for sub-node:%a\n",
              Table.Property, NodeName));
    } else {
      DEBUG ((EFI_D_VERBOSE, "Property:%a found for sub-node:%a\tProperty:%a\n",
              Table.Property, NodeName, Prop->data));

      /* For Dynamic partition support disable firmware fstab nodes. */
      if (IsDynamicPartitionSupport ()) {
        DEBUG ((EFI_D_VERBOSE, "Disabling node status :%a\n", NodeName));
        Status = fdt_setprop (fdt, SubNodeOffset, Table.Property, "disabled",
                             (AsciiStrLen ("disabled") + 1));
        if (Status) {
         DEBUG ((EFI_D_ERROR, "ERROR: Failed to disable Node: %a\n", NodeName));
        }
        continue;
      }

      /* Pointer to fdt 'dev' property string that needs to update based on the
       * 'androidboot.bootdevice' */
      ReplaceStr = (CHAR8 *)Prop->data;
      ReplaceStr = AsciiStrStr (ReplaceStr, Table.DevicePathId);
      if (!ReplaceStr) {
        DEBUG ((EFI_D_VERBOSE, "Update property:%a value is not proper to "
                               "update for sub-node:%a\n",
                Table.Property, NodeName));
        continue;
      }
      ReplaceStr += AsciiStrLen (Table.DevicePathId);
      NextStr = AsciiStrStr ((ReplaceStr + 1), "/");
      DevNodeBootDevLen = NextStr - ReplaceStr;
      if (DevNodeBootDevLen >= AsciiStrLen (BootDevBuf)) {
        gBS->CopyMem (ReplaceStr, BootDevBuf, AsciiStrLen (BootDevBuf));
        PaddingEnd = DevNodeBootDevLen - AsciiStrLen (BootDevBuf);
        /* Update the property with new value */
        if (PaddingEnd) {
          gBS->CopyMem (ReplaceStr + AsciiStrLen (BootDevBuf), NextStr,
                        AsciiStrLen (NextStr));
          for (Index = 0; Index < PaddingEnd; Index++) {
            ReplaceStr[AsciiStrLen (BootDevBuf) + AsciiStrLen (NextStr) +
                       Index] = ' ';
          }
        }
      } else {
        DEBUG ((EFI_D_ERROR, "String length mismatch b/w DT Bootdevice string"
                             " (%d) and expected Bootdevice strings (%d)\n",
                DevNodeBootDevLen, AsciiStrLen (BootDevBuf)));
      }
    }
  }

  if (BootDevBuf) {
    FreePool (BootDevBuf);
  }
  BootDevBuf = NULL;
  return Status;
}
