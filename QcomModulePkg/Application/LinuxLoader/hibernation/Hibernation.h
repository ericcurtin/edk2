/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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
#if HIBERNATION_SUPPORT

#define __aligned(x)	__attribute__((aligned(x)))
#define __packed	__attribute__((packed))
#define __AC(X,Y)       (X##Y)
#define _AC(X,Y)        __AC(X,Y)
#define PAGE_SHIFT	12
#define PAGE_SIZE	(_AC(1, UL) << PAGE_SHIFT)
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define HIBERNATE_SIG           "S1SUSPEND"
#define __NEW_UTS_LEN 64

/* Return True if integer overflow will occur */
#define CHECK_ADD64(a, b) ((MAX_UINT64 - b < a) ? TRUE : FALSE)
#define printf(...)     DEBUG((EFI_D_ERROR, __VA_ARGS__))

EFI_STATUS
PartitionGetInfo (IN CHAR16 *PartitionName,
		  OUT EFI_BLOCK_IO_PROTOCOL **BlockIo,
		  OUT EFI_HANDLE **Handle);
void JumpToKernel (void);
typedef VOID (*HIBERNATION_KERNEL)(VOID);
typedef unsigned int u32;
typedef unsigned long sector_t;

struct swsusp_header {
	char reserved[PAGE_SIZE - 20 - sizeof(sector_t) - sizeof(int) -
		sizeof(u32)];
	u32     crc32;
	sector_t image;
	unsigned int flags;     /* Flags to pass to the "boot" kernel */
	char    orig_sig[10];
	char    sig[10];
} __packed;

struct new_utsname {
	char sysname[__NEW_UTS_LEN + 1];
	char nodename[__NEW_UTS_LEN + 1];
	char release[__NEW_UTS_LEN + 1];
	char version[__NEW_UTS_LEN + 1];
	char machine[__NEW_UTS_LEN + 1];
	char domainname[__NEW_UTS_LEN + 1];
};

struct swsusp_info {
	struct new_utsname	uts;
	unsigned int		rsion_code;
	unsigned long		num_physpages;
	int			cpus;
	unsigned long		image_pages;
	unsigned long		pages;
	unsigned long		size;
} __aligned(PAGE_SIZE);

struct arch_hibernate_hdr_invariants {
        char            uts_version[__NEW_UTS_LEN + 1];
};

struct arch_hibernate_hdr {
	struct arch_hibernate_hdr_invariants invariants;
	unsigned long	ttbr1_el1;
	void            (*reenter_kernel)(void);
	unsigned long	phys_reenter_kernel;
	unsigned long	 __hyp_stub_vectors;
	unsigned long	sleep_cpu_mpidr;
};
#endif
