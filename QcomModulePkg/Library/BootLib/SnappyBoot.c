/**
 * Copyright (C) 2019 Canonical Ltd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
  * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <Uefi.h>
#include <Library/BaseLib.h>
#include <Library/Board.h>
#include <Library/BootLinux.h>
#include <Library/LinuxLoaderLib.h>
#include <Library/UefiLib.h>
#include <Uefi/UefiSpec.h>
#include <VerifiedBoot.h>
#include <Library/SnappyBoot.h>

static CHAR8 cmdline_buf[512];

static uint32_t crc32(uint32_t crc, unsigned char *buf, size_t len)
{
    int k;

    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
    }
    return ~crc;
}

static EFI_STATUS MapBootimg(
                       CHAR8 BootimgMatrix[][2][SNAP_NAME_MAX_LEN],
                       UINT32 Rows,
                       Slot *BootableSlot,
                       const CHAR8 *KernelSnap
                     );
static EFI_STATUS LoadRunEnvironment(SNAP_RUN_BOOT_SELECTION_t **BootSelect );
static EFI_STATUS LoadRecoveryEnvironment(
                             SNAP_RECOVERY_BOOT_SELECTION_t **RecoverySelect
                           );

static EFI_STATUS SaveRunEnvironment(
              SNAP_RUN_BOOT_SELECTION_t *BootSelect
            );
// static EFI_STATUS SaveRecoveryEnvironment(
//               SNAP_RECOVERY_BOOT_SELECTION_t *RecoverySelect);
static EFI_STATUS LoadRunEnvironmentFromPart(
              const CHAR8 *partName,
              SNAP_RUN_BOOT_SELECTION_t **BootSelect
            );
static EFI_STATUS LoadRecoveryEnvironmentFromPart(
              const CHAR8 *partName,
              SNAP_RECOVERY_BOOT_SELECTION_t **RecoverySelect
            );
static EFI_STATUS LoadEnvImageFromPartition(
              VOID **EnvImage,
              UINT32 ImageSize,
              const CHAR8 *partName
            );
static EFI_STATUS SaveEnvImageToPartition (
              VOID *EnvImage,
              UINT32 ImageSize,
              const CHAR8 *partName
            );

EFI_STATUS SnapGetTargetBootParams(Slot *BootableSlot,
                                   CHAR8 **cmdline,
                                   BOOLEAN unlocked
                                 )
{
    EFI_STATUS Status = EFI_SUCCESS;
    CHAR8 *snap_kernel = NULL;
    CHAR8 *recovery_mode = NULL;
    CHAR8 *dangerous;
    SNAP_RUN_BOOT_SELECTION_t *BootSelect = NULL;
    SNAP_RECOVERY_BOOT_SELECTION_t *RecoverySelect = NULL;

    Status = LoadRecoveryEnvironment(&RecoverySelect);
    if (Status != EFI_SUCCESS || RecoverySelect == NULL){
        DEBUG ((EFI_D_ERROR, "snap: missing recovery environment!\n"));
        return Status;
    }

    if (unlocked) {
        dangerous = " dangerous";
    } else {
        dangerous = "";
    }

    // recovery mode can be unset, in which case "install" should be assumed
    if (AsciiStrLen(RecoverySelect->snapd_recovery_mode)) {
        recovery_mode = RecoverySelect->snapd_recovery_mode;
    } else {
        recovery_mode = SNAP_RECOVERY_MODE_INSTALL;
    }

    if ( AsciiStrnCmp(recovery_mode, SNAP_RECOVERY_MODE_RUN, SNAP_NAME_MAX_LEN)) {
        DEBUG((EFI_D_INFO, "snap: get boot params: detected %a mode\n",
                           recovery_mode ));
        AsciiSPrint(cmdline_buf,
                    sizeof(cmdline_buf),
                    " snapd_recovery_mode=%a snapd_recovery_system=%a%a",
                    recovery_mode,
                    RecoverySelect->snapd_recovery_system,
                    dangerous
                  );

        // map recovery system to boot partition
        MapBootimg(
            RecoverySelect->bootimg_matrix,
            SNAP_RECOVERY_BOOTIMG_PART_NUM,
            BootableSlot,
            RecoverySelect->snapd_recovery_system
          );
        *cmdline = cmdline_buf;
        // cleanup and boot recovery
        goto cleanup;
    }

    AsciiSPrint(cmdline_buf,
                sizeof(cmdline_buf),
                " snapd_recovery_mode=%a%a",
                recovery_mode,
                dangerous
              );

    // load run time environment to handle run time case
    Status = LoadRunEnvironment(&BootSelect);

    if (Status != EFI_SUCCESS || BootSelect == NULL) {
        DEBUG ((EFI_D_INFO, "SnapGetTargetBootParams no boot env loaded\n"));
        return Status;
    }

    snap_kernel = BootSelect->snap_kernel;
    if ( !AsciiStrnCmp(BootSelect->kernel_status,
                       SNAP_MODE_TRY,
                       SNAP_NAME_MAX_LEN
                     )
       ) {
        AsciiStrCpyS(BootSelect->kernel_status,
                     SNAP_NAME_MAX_LEN,
                     SNAP_MODE_TRYING
                   );
        Status = SaveRunEnvironment(BootSelect);
        if (EFI_SUCCESS != Status)
            goto cleanup;
        if (AsciiStrLen( BootSelect->snap_try_kernel)) {
            snap_kernel = BootSelect->snap_try_kernel;
        }
    } else if ( !AsciiStrnCmp(
                 BootSelect->kernel_status,
                 SNAP_MODE_TRYING,
                 SNAP_NAME_MAX_LEN)
              ) {
        BootSelect->kernel_status[0] = 0;
        Status = SaveRunEnvironment(BootSelect);
        if (EFI_SUCCESS != Status)
            goto cleanup;
    }

    // map runtime bootimage to the partition
    MapBootimg(BootSelect->bootimg_matrix,
               SNAP_RUN_BOOTIMG_PART_NUM,
               BootableSlot,
               snap_kernel
             );
    *cmdline = cmdline_buf;
    cleanup:
        // if(RecoverySelect) {
        //     FreePool(RecoverySelect);
        // }
        // if (BootSelect != NULL) {
        //     FreePool(BootSelect);
        // }
        DEBUG ((EFI_D_INFO, "SnapGetTargetBootParams skip cleanup\n"));
        return Status;
}

EFI_STATUS MapBootimg( CHAR8 BootimgMatrix[][2][SNAP_NAME_MAX_LEN],
                        UINT32 Rows,
                        Slot *BootableSlot,
                        const CHAR8 *KernelSnap
                      )
{
    CHAR8 *slot;
    for (size_t n = 0; n < Rows; ++n) {
        if (!AsciiStrnCmp(BootimgMatrix[n][1],
            KernelSnap,
            SNAP_NAME_MAX_LEN
          )
        ) {
            if (AsciiStrLen(BootimgMatrix[n][0])) {
                // we need only slot part of partition table:( '_a','_b','_ra','_rb' )
                slot = AsciiStrStr(BootimgMatrix[n][0], "_");
                if (slot) {
                    AsciiStrToUnicodeStr(slot, BootableSlot->Suffix);
                    return EFI_SUCCESS;
                } else {
                    DEBUG ((EFI_D_INFO, "MapBootimg missing partition sufix\n"));
                    return EFI_NOT_FOUND;
                }
            }
        }
    }
    DEBUG ((EFI_D_ERROR, "snap: MapBootimg: did not find valid boot partition!!!\n"));
    return EFI_NOT_FOUND;
}

// load runtime environment, try to load backup env if main fails
static EFI_STATUS LoadRunEnvironment(SNAP_RUN_BOOT_SELECTION_t **BootSelect )
{
    EFI_STATUS Status = EFI_SUCCESS;
    Status = LoadRunEnvironmentFromPart(SNAP_BOOTSELECT_PARTITION,
                                            BootSelect
                                          );
    if (*BootSelect == NULL) {
        Status = LoadRunEnvironmentFromPart(SNAP_BOOTSELECT_PARTITION "bak",
                                                BootSelect
                                              );
        // if we successfully loaded backup env, try to save it to main, ignore error
        if (*BootSelect)
            // restore primary environment
            SaveEnvImageToPartition( (VOID *)*BootSelect,
                                     sizeof(SNAP_RUN_BOOT_SELECTION_t),
                                     SNAP_BOOTSELECT_PARTITION
                                   );
    }
    return Status;
}

// load recovery environment, try to load backup env if main fails
static EFI_STATUS LoadRecoveryEnvironment(
                             SNAP_RECOVERY_BOOT_SELECTION_t **RecoverySelect
                           )
{
    EFI_STATUS Status = EFI_SUCCESS;
    Status = LoadRecoveryEnvironmentFromPart(SNAP_RECOVERYSELECT_PARTITION,
                                             RecoverySelect
                                           );
    if (*RecoverySelect == NULL) {
        Status = LoadRecoveryEnvironmentFromPart(
                                           SNAP_RECOVERYSELECT_PARTITION "bak",
                                           RecoverySelect
                                         );
        // if we successfully loaded backup env, try to save it to main, ignore error
        if (*RecoverySelect) {
            // restore primary environment
            SaveEnvImageToPartition( (VOID *)*RecoverySelect,
                                     sizeof(SNAP_RECOVERY_BOOT_SELECTION_t),
                                     SNAP_RECOVERYSELECT_PARTITION
                                   );
        }
    }
    return Status;
}

// save runtime environment always to main and backup env
static EFI_STATUS SaveRunEnvironment(SNAP_RUN_BOOT_SELECTION_t *BootSelect)
{
    // first calculate crc32 for the passed boot selection
    BootSelect->crc32 = crc32( 0,
                             (unsigned char *)BootSelect,
                             sizeof(SNAP_RUN_BOOT_SELECTION_t) - sizeof(uint32_t)
                           );
    // if at least one write works, return success, use two variables, that compiler
    // does not optimise
    EFI_STATUS r = SaveEnvImageToPartition( (VOID *)BootSelect,
                                            sizeof(SNAP_RUN_BOOT_SELECTION_t),
                                            SNAP_BOOTSELECT_PARTITION
                                          );
    EFI_STATUS rb = SaveEnvImageToPartition( (VOID *)BootSelect,
                                             sizeof(SNAP_RUN_BOOT_SELECTION_t),
                                             SNAP_BOOTSELECT_PARTITION "bak"
                                           );
    return r == EFI_SUCCESS ? rb : r;
}

// // save recovery environment always to main and backup env
// static EFI_STATUS SaveRecoveryEnvironment(
//                                SNAP_RECOVERY_BOOT_SELECTION_t *RecoverySelect)
// {
//     // first calculate crc32 for the passed boot selection
//     RecoverySelect->crc32 = crc32( 0,
//                       (unsigned char *)RecoverySelect,
//                       sizeof(SNAP_RECOVERY_BOOT_SELECTION_t) - sizeof(uint32_t));
//
//     // if at least one write works, return success, use two variables, that compiler
//     // does not optimise
//     int r = SaveEnvImageToPartition( (VOID *)RecoverySelect,
//                                   sizeof(SNAP_RECOVERY_BOOT_SELECTION_t),
//                                   SNAP_RECOVERYSELECT_PARTITION);
//     int rb = SaveEnvImageToPartition( (VOID *)RecoverySelect,
//                                   sizeof(SNAP_RECOVERY_BOOT_SELECTION_t),
//                                   SNAP_RECOVERYSELECT_PARTITION "bak");
//     return r & rb;
// }

static EFI_STATUS LoadRunEnvironmentFromPart(const CHAR8 *partName,
                                        SNAP_RUN_BOOT_SELECTION_t **BootSelect
                                      )
{
    EFI_STATUS Status = EFI_SUCCESS;
    UINT32 crc = 0;
    // BlockIo = HandleInfoList[0].BlkIo;
    // BlockIo->Media->BlockSize;
    DEBUG ((EFI_D_INFO, "snap: aligned size %d\n",
                        ALIGN_PAGES (sizeof(SNAP_RUN_BOOT_SELECTION_t),
                        ALIGNMENT_MASK_4KB)));
    Status = LoadEnvImageFromPartition( (VOID**)BootSelect,
                                        sizeof(SNAP_RUN_BOOT_SELECTION_t),
                                        partName
                                      );
    if (Status != EFI_SUCCESS && *BootSelect == NULL) {
       return Status;
    }

    if ( (*BootSelect)->version != SNAP_BOOTSELECT_VERSION_V2
         || (*BootSelect)->signature != SNAP_BOOTSELECT_SIGNATURE_RUN )
    {
        DEBUG ((EFI_D_ERROR,
          "snap: LoadRunEnvironmentFromPart(%a): ERROR version/signature broken [0x%X] vs [0x%X], [0x%X] vs [0x%X]!\n",
                partName,
                (*BootSelect)->version,
                SNAP_BOOTSELECT_VERSION_V2,
                (*BootSelect)->signature,
                SNAP_BOOTSELECT_SIGNATURE_RUN));
        Status = EFI_INCOMPATIBLE_VERSION;
        goto cleanup;
    }
    crc = crc32( 0,
                 (unsigned char *)(*BootSelect),
                 sizeof(SNAP_RUN_BOOT_SELECTION_t)-sizeof(uint32_t)
               );
    if ( (*BootSelect)->crc32 != crc )
    {
        DEBUG ((EFI_D_ERROR,
          "snap: LoadRunEnvironmentFromPart(%a): ERROR crc32 broken [0x%X] vs [0x%X]!\n",
          partName,
          (*BootSelect)->crc32,
          crc ));
        Status = EFI_CRC_ERROR;
        goto cleanup;
    } else {
        DEBUG ((EFI_D_INFO,
          "snap: LoadRunEnvironmentFromPart(%a): crc32 sucessfully validated\n",
          partName ));
    }
    return Status;

    cleanup:
        if (*BootSelect) {
            FreePool(*BootSelect);
        }
        *BootSelect = NULL;
        return Status;
}

static EFI_STATUS LoadRecoveryEnvironmentFromPart(const CHAR8 *partName,
                              SNAP_RECOVERY_BOOT_SELECTION_t **RecoverySelect
                            )
{
    EFI_STATUS Status = EFI_SUCCESS;
    UINT32 crc = 0;
    // BlockIo = HandleInfoList[0].BlkIo;
    // BlockIo->Media->BlockSize;
    DEBUG ((EFI_D_INFO, "snap: aligned size %d\n",
                        ALIGN_PAGES (sizeof(SNAP_RECOVERY_BOOT_SELECTION_t), ALIGNMENT_MASK_4KB)));
    Status = LoadEnvImageFromPartition( (VOID**)RecoverySelect,
                                     sizeof(SNAP_RECOVERY_BOOT_SELECTION_t),
                                     partName
                                   );
    if (Status != EFI_SUCCESS) {
       return Status;
    }

    if ( (*RecoverySelect)->version != SNAP_BOOTSELECT_VERSION_V2
         || (*RecoverySelect)->signature != SNAP_BOOTSELECT_SIGNATURE_RECOVERY )
    {
        DEBUG ((EFI_D_ERROR,
          "snap: LoadRecoveryEnvironmentFromPart(%a): ERROR version/signature broken [0x%X] vs [0x%X], [0x%X] vs [0x%X]!\n",
                partName,
                (*RecoverySelect)->version,
                SNAP_BOOTSELECT_VERSION_V2,
                (*RecoverySelect)->signature,
                SNAP_BOOTSELECT_SIGNATURE_RECOVERY));
        Status = EFI_INCOMPATIBLE_VERSION;
        goto cleanup;
    }
    crc = crc32( 0,
                 (unsigned char *)(*RecoverySelect),
                 sizeof(SNAP_RECOVERY_BOOT_SELECTION_t)-sizeof(uint32_t)
               );
    if ( (*RecoverySelect)->crc32 != crc )
    {
        DEBUG ((EFI_D_ERROR,
          "snap: LoadRecoveryEnvironmentFromPart(%a): ERROR crc32 broken [0x%X] vs [0x%X]!\n",
          partName,
          (*RecoverySelect)->crc32,
          crc ));
        Status = EFI_CRC_ERROR;
        goto cleanup;
    } else {
        DEBUG ((EFI_D_INFO,
          "snap: LoadRecoveryEnvironmentFromPart(%a): crc32 sucessfully validated\n",
          partName ));
    }
    return Status;

    cleanup:
        if (*RecoverySelect) {
            FreePool(*RecoverySelect);
        }
        *RecoverySelect = NULL;
        return Status;
}

static EFI_STATUS LoadEnvImageFromPartition(VOID **EnvImage,
                                            UINT32 ImageSize,
                                            const CHAR8 *partName)
{
    EFI_STATUS Status = EFI_SUCCESS;
    UINT32 ActualSize = ImageSize + 60;
    VOID *image = NULL;
    CHAR16 Pname[MAX_GPT_NAME_SIZE];

    // BlockIo = HandleInfoList[0].BlkIo;
    // BlockIo->Media->BlockSize;
    DEBUG ((EFI_D_INFO, "snap: aligned env size %d\n",
                        ALIGN_PAGES (ActualSize, ALIGNMENT_MASK_4KB)));
    image = (VOID *) AllocatePages (ALIGN_PAGES (ActualSize, ALIGNMENT_MASK_4KB));
    if (image == NULL) {
      DEBUG ((EFI_D_ERROR, "snap: Failed to allocate zero pool for EnvImage\n"));
      return EFI_OUT_OF_RESOURCES;
    }

    AsciiStrToUnicodeStr(partName, Pname);
    Status = LoadImageFromPartition((VOID *)(image),
                                    (UINT32 *)&(ActualSize),
                                    Pname
                                  );
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "ERROR: Failed to load image from partition: %r\n",
              Status));
      goto cleanup;
    }

    DEBUG ((EFI_D_INFO,
             "snap: LoadEnvImageFromPartition(%a): read SNAP_BOOT_SELECTION: SUCCESS, read %d bytes\n",
             Pname,
             ActualSize));

    *EnvImage = image;
    return Status;

    cleanup:
        if ( image ) {
            FreePool(image);
        }
        return Status;
}

static EFI_STATUS SaveEnvImageToPartition (VOID *EnvImage,
                                           UINT32 ImageSize,
                                           const CHAR8 *partName
                                         )
{
    EFI_STATUS Status;
    EFI_BLOCK_IO_PROTOCOL *BlkIo;
    PartiSelectFilter HandleFilter;
    HandleInfo HandleInfoList[1];
    STATIC UINT32 MaxHandles;
    STATIC UINT32 BlkIOAttrib = 0;
    EFI_HANDLE *Handle = NULL;
    CHAR16 Pname[MAX_GPT_NAME_SIZE];

    AsciiStrToUnicodeStr(partName, Pname);

    BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
    BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
    BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
    BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_LABEL;

    HandleFilter.RootDeviceType = NULL;
    HandleFilter.PartitionLabel = Pname;
    HandleFilter.VolumeName = NULL;

    if (EnvImage == NULL) {
        DEBUG ((EFI_D_ERROR, "snap: SaveEnvImageToPartition: invalid EnvImage\n"));
    }

    DEBUG ((DEBUG_INFO, "snap: SaveEnvImageToPartition: Start : %u ms\n", GetTimerCountms ()));

    MaxHandles = sizeof (HandleInfoList) / sizeof (*HandleInfoList);

    Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

    if (Status == EFI_SUCCESS) {
        if (MaxHandles == 0) {
            DEBUG ((EFI_D_ERROR, "ERROR: snap: no partion found [%s] \n", Pname));
            return EFI_NO_MEDIA;
        }
        if (MaxHandles != 1) {
            // Unable to deterministically load from single partition
            DEBUG ((EFI_D_INFO, "ExecImgFromVolume(): multiple partitions found.\r\n"));
            return EFI_LOAD_ERROR;
        }
    } else {
        DEBUG ((EFI_D_ERROR, "%s: GetBlkIOHandles failed: %r\n", __func__, Status));
        return Status;
    }

    BlkIo = HandleInfoList[0].BlkIo;
    Handle = HandleInfoList[0].Handle;
    Status = WriteBlockToPartition (BlkIo, Handle, 0, ImageSize, EnvImage);
    if (Status != EFI_SUCCESS) {
        DEBUG ((EFI_D_ERROR, "SaveEnvImageToPartition: failed :%r\n", Status));
    }

    return Status;
}
