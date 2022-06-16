/* Copyright (c) 2021 Canonical Ltd
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
 *   * Neither the name of Canonical Ltd nor the names of its
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

#ifndef _BOOTLOADER_SNAP_BOOT_COMMON_H
#define _BOOTLOADER_SNAP_BOOT_COMMON_H

#define SNAP_BOOTSELECT_SIGNATURE ('S' | ('B' << 8) | ('s' << 16) | ('e' << 24))
// SNAP_BOOTSELECT_SIGNATURE_RUN is the same as SNAP_BOOTSELECT_SIGNATURE
#define SNAP_BOOTSELECT_SIGNATURE_RUN ('S' | ('B' << 8) | ('s' << 16) | ('e' << 24))

// note SNAP_NAME_MAX_LEN also defines the max length of a recovery system label
#define SNAP_NAME_MAX_LEN (256)
#define HASH_LENGTH (32)
#define SNAP_MODE_TRY "try"
#define SNAP_MODE_TRYING "trying"
#define FACTORY_RESET "factory-reset"

#define SNAP_RECOVERY_MODE_INSTALL "install"
#define SNAP_RECOVERY_MODE_RUN "run"
#define SNAP_RECOVERY_MODE_RECOVER "recover"

/* partition label where boot select structure is stored, for uc20 this is just
 * used for run mode
 */
#define SNAP_BOOTSELECT_PARTITION "snapbootsel"

/* partition label where recovery boot select structure is stored */
#define SNAP_RECOVERYSELECT_PARTITION "snaprecoverysel"

/** maximum number of available bootimg partitions for recovery systems, min 5
 *  NOTE: the number of actual bootimg partitions usable is determined by the
 *  gadget, this just sets the upper bound of maximum number of recovery systems
 *  a gadget could define without needing changes here
 */
#define SNAP_RECOVERY_BOOTIMG_PART_NUM 10

/* number of available bootimg partitions for run mode, min 2 */
#define SNAP_RUN_BOOTIMG_PART_NUM 2

#endif  // _BOOTLOADER_SNAP_BOOT_COMMON_H
