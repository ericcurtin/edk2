/*
 * Copyright (c) 2016-2017,2019, The Linux Foundation. All rights reserved.
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

#include "BootStats.h"
#include "AutoGen.h"
#include "BootLinux.h"
#include "Reg.h"

#define BS_INFO_OFFSET (0x6B0)

STATIC UINT32 BootLoadStart;
STATIC UINT32 BootLoadEnd;
STATIC UINT32 KernelEntry;
STATIC UINT32 KernelLoadStart;
STATIC UINT32 KernelLoadDone;
STATIC UINT32 KernelAuthStart;
STATIC UINT32 KernelAuthDone;
STATIC UINT64 SharedImemAddress;
STATIC UINT64 MpmTimerBase;
STATIC UINT64 BsImemAddress;

void
BootStatsSetTimeStamp (BS_ENTRY BootStatId)
{
  EFI_STATUS Status;
  UINT32 BootStatClockCount = 0;

  UINTN DataSize = sizeof (SharedImemAddress);

  if (!SharedImemAddress) {
    Status =
        gRT->GetVariable ((CHAR16 *)L"Shared_IMEM_Base", &gQcomTokenSpaceGuid,
                          NULL, &DataSize, &SharedImemAddress);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Failed to get Shared IMEM base, %r\n", Status));
      return;
    }
  }

  DataSize = sizeof (MpmTimerBase);
  if (!MpmTimerBase) {
    Status =
        gRT->GetVariable ((CHAR16 *)L"MPM2_SLP_CNTR_ADDR", &gQcomTokenSpaceGuid,
                          NULL, &DataSize, &MpmTimerBase);
    if (Status != EFI_SUCCESS) {
      DEBUG (
          (EFI_D_ERROR, "Failed to get MPM Sleep counter base, %r\n", Status));
      return;
    }
  }

  if (SharedImemAddress && MpmTimerBase) {
    UINT64 BootStatImemAddress;
    BsImemAddress = SharedImemAddress + BS_INFO_OFFSET;

    if (BootStatId >= BS_MAX) {
      DEBUG (
          (EFI_D_ERROR, "Bad BootStat id: %u, Max: %u\n", BootStatId, BS_MAX));
      return;
    }

    if (BootStatId == BS_BL_START) {
      BootLoadStart = READL (MpmTimerBase);
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Bootloader Start:%u\n",
              BootStatId, BootLoadStart));
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * BS_BL_START);
       if (BootLoadStart) {
        WRITEL (BootStatImemAddress, BootLoadStart);
      }
      return;
    }

    if (BootStatId == BS_KERNEL_LOAD_START) {
      KernelLoadStart = READL (MpmTimerBase);
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Kernel Load Start:%u\n",
              BootStatId, KernelLoadStart));
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * BS_KERNEL_LOAD_START);
       if (KernelLoadStart) {
        WRITEL (BootStatImemAddress, KernelLoadStart);
      }
      return;
    }

    if (BootStatId == BS_KERNEL_LOAD_DONE) {
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * BS_KERNEL_LOAD_DONE);
      KernelLoadDone = READL (MpmTimerBase);
      if (KernelLoadDone) {
        WRITEL (BootStatImemAddress, KernelLoadDone);
      }
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Kernel Load Done:%u\n",
	       BootStatId, KernelLoadDone));
      return;
    }

    if (BootStatId == BS_BOOTIMAGE_CHECKSUM_START) {
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * BS_BOOTIMAGE_CHECKSUM_START);
      KernelAuthStart = READL (MpmTimerBase);
      if (KernelAuthStart) {
	WRITEL (BootStatImemAddress, KernelAuthStart);
      }
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Kernel Auth Start:%u\n",
              BootStatId, KernelAuthStart));
      return;
    }

    if (BootStatId == BS_BOOTIMAGE_CHECKSUM_DONE) {
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * (BS_BOOTIMAGE_CHECKSUM_DONE));
      KernelAuthDone = READL (MpmTimerBase);
      if (KernelAuthDone) {
        WRITEL (BootStatImemAddress, KernelAuthDone);
      }
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Kernel Auth Done:%u\n",
              BootStatId, KernelAuthDone));
      return;
    }

    if (BootStatId == BS_BL_END) {
      BootLoadEnd = READL (MpmTimerBase);
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Bootloader End:%u\n",
              BootStatId, BootLoadEnd));
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * BS_BL_END);
       if (BootLoadEnd) {
        WRITEL (BootStatImemAddress, BootLoadEnd);
      }
      return;
    }

    if (BootStatId == BS_KERNEL_ENTRY) {
      KernelEntry = READL (MpmTimerBase);
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: Kernel Entry:%u\n",
              BootStatId, KernelEntry));
      BootStatImemAddress =
          BsImemAddress + (sizeof (UINT32) * BS_KERNEL_ENTRY);
       if (KernelEntry) {
        WRITEL (BootStatImemAddress, KernelEntry);
      }
      return;
    } else {
      BootStatImemAddress = BsImemAddress + (sizeof (UINT32) * BootStatId);
      BootStatClockCount = READL (MpmTimerBase);
      if (BootStatClockCount) {
        WRITEL (BootStatImemAddress, BootStatClockCount);
      }
      DEBUG ((EFI_D_VERBOSE, "BootStats: ID-%d: BootStatClockCount:%u\n",
              BootStatId, BootStatClockCount));
    }
  }
}
