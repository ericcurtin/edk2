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

#include <Library/DeviceInfo.h>
#include <Library/DrawUI.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/ShutdownServices.h>
#include <Library/StackCanary.h>
#include "Hibernation.h"

static struct free_ranges free_range_buf[100];
static int buf_index;
static unsigned int nr_copy_pages;
static unsigned int nr_meta_pages;
static UINT64 bounce_base_addr = 0x140000000;
static struct bounce_pfn_entry *bounce_pfn_entry_table;
static struct bounce_pfn_entry *next_bounce_pfn_entry;
static unsigned long bounced_pages;
static unsigned long bounce_entry_count;
static void *next_bounce_buf;
UINT64 relocation_base_addr;
static struct swsusp_header *swsusp_header;
static struct arch_hibernate_hdr *resume_hdr;
static struct swsusp_info *swsusp_info;

static int memcmp(const void *s1, const void *s2, int n)
{
	const unsigned char *us1 = s1;
	const unsigned char *us2 = s2;

	if (n == 0 )
		return 0;

	while(n--) {
		if(*us1++ ^ *us2++)
			return 1;
	}
	return 0;
}

static void copyPage(unsigned long src_pfn, unsigned long dst_pfn)
{
	unsigned long *src = (unsigned long*)(src_pfn << PAGE_SHIFT);
	unsigned long *dst = (unsigned long*)(dst_pfn << PAGE_SHIFT);

	gBS->CopyMem (dst, src, PAGE_SIZE);
}

static int get_uefi_memory_map(void)
{
	EFI_MEMORY_DESCRIPTOR	*MemMap;
	EFI_MEMORY_DESCRIPTOR	*MemMapPtr;
	UINTN			MemMapSize;
	UINTN			MapKey, DescriptorSize;
	UINTN			Index;
	UINT32			DescriptorVersion;
	EFI_STATUS		Status;

	MemMapSize = 0;
	MemMap     = NULL;

	Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
				&DescriptorSize, &DescriptorVersion);
	if (Status != EFI_BUFFER_TOO_SMALL) {
		DEBUG ((EFI_D_ERROR, "ERROR: Undefined response get memory map\n"));
		return -1;
	}
	if (CHECK_ADD64 (MemMapSize, EFI_PAGE_SIZE)) {
		DEBUG ((EFI_D_ERROR, "ERROR: integer Overflow while adding additional"
					"memory to MemMapSize"));
		return -1;
	}
	MemMapSize = MemMapSize + EFI_PAGE_SIZE;
	MemMap = AllocateZeroPool (MemMapSize);
	if (!MemMap) {
		DEBUG ((EFI_D_ERROR,
			"ERROR: Failed to allocate memory for memory map\n"));
		return -1;
	}
	MemMapPtr = MemMap;
	Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
				&DescriptorSize, &DescriptorVersion);
	if (EFI_ERROR (Status)) {
		DEBUG ((EFI_D_ERROR, "ERROR: Failed to query memory map\n"));
		FreePool (MemMapPtr);
		return -1;
	}
	for (Index = 0; Index < MemMapSize / DescriptorSize; Index ++) {
		if (MemMap->Type == EfiConventionalMemory) {
			int ret;
			free_range_buf[buf_index].start = MemMap->PhysicalStart;
			free_range_buf[buf_index].end =  MemMap->PhysicalStart + MemMap->NumberOfPages * 4096;
			/* TODO: Remove below address dependency */
			if (MemMap->PhysicalStart != 0x14FAFB000) {
				ret = gBS->AllocatePages(AllocateAddress, EfiBootServicesData,
						MemMap->NumberOfPages, &MemMap->PhysicalStart);
				if(ret)
					printf("error alloc LINE %d\n", __LINE__);
			}
			DEBUG ((EFI_D_ERROR, "Free Range 0x%lx --- 0x%lx\n",free_range_buf[buf_index].start,
			free_range_buf[buf_index].end));
			buf_index++;
		}
		MemMap = (EFI_MEMORY_DESCRIPTOR *)((UINTN)MemMap + DescriptorSize);
	}
	FreePool (MemMapPtr);
	return 0;
}

static int read_image(unsigned long offset, VOID *Buff, int nr_pages) {

	int Status;
	EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
	EFI_HANDLE *Handle = NULL;

	Status = PartitionGetInfo (L"system_b", &BlockIo, &Handle);
	if (Status != EFI_SUCCESS)
		return Status;

	if (!Handle) {
		DEBUG ((EFI_D_ERROR, "EFI handle for system_b is corrupted\n"));
		return -1;
	}

	if (CHECK_ADD64 (BlockIo->Media->LastBlock, 1)) {
		DEBUG ((EFI_D_ERROR, "Integer overflow while adding LastBlock and 1\n"));
		return -1;
	}

	if ((MAX_UINT64 / (BlockIo->Media->LastBlock + 1)) <
			(UINT64)BlockIo->Media->BlockSize) {
		DEBUG ((EFI_D_ERROR,
			"Integer overflow while multiplying LastBlock and BlockSize\n"));
		return -1;
	}

	/* Check what is the block size of the mmc and scale the offset accrodingly
	 * right now blocksize = page_size = 4096 */

	Status = BlockIo->ReadBlocks (BlockIo,
			BlockIo->Media->MediaId,
			offset,
			EFI_PAGE_SIZE * nr_pages,
			(VOID*)Buff);
	if (Status != EFI_SUCCESS) {
		printf("Read image failed Line = %d\n", __LINE__);
		return Status;
	}

	return 0;
}

/*
 * target_addr  : address where page allocation is needed
 *
 * return	: 1 if address falls in free range
 * 		  0 if address is not in free range
 */
static int CheckFreeRanges (UINT64 target_addr)
{
	int i = 0;
	while (i < buf_index) {
		if (target_addr >= free_range_buf[i].start &&
			target_addr < free_range_buf[i].end)
		return 1;
		i++;
	}
	return 0;
}

/*
 * Function return 1 if page is bounced and 0 otherwise.
 */
static int copy_page_to_dst(unsigned long src_pfn, unsigned long dst_pfn, unsigned long bounce_pfn)
{
	int ret = 1;
	UINT64 target_addr = dst_pfn << PAGE_SHIFT;

	if (CheckFreeRanges(target_addr)) {
		if (target_addr > 0x14FAFB000 && target_addr < 0x150000000)
			gBS->AllocatePages(AllocateAddress, EfiBootServicesData, 1, &target_addr);
		copyPage(src_pfn, dst_pfn);
		ret = 0;
	} else
		copyPage(src_pfn, bounce_pfn);
	return ret;
}

static void update_bounce_entry(unsigned long pfn)
{
	unsigned long pfn_end = pfn + 1;

	if ((pfn >= (0x140000000 << PAGE_SHIFT)) && ( pfn <= (relocation_base_addr << PAGE_SHIFT)))
		printf("Error: Overlap pfn = %lu\n", pfn);

	if ((pfn_end >= (0x140000000 << PAGE_SHIFT)) && ( pfn_end <= (relocation_base_addr << PAGE_SHIFT)))
		printf("Error: Overlap end pfn = %lu\n", pfn_end);

	next_bounce_pfn_entry->dst_pfn = pfn;
	next_bounce_pfn_entry->pages = 1;
	next_bounce_pfn_entry++;
	next_bounce_buf += (1 << PAGE_SHIFT);
	bounced_pages ++;
	bounce_entry_count++;

	if (bounced_pages > MAX_BOUNCE_PAGES)
		printf("================= Error: Bounce buffers exceeded the limit==============\n");
}

static void print_kernel_details(struct swsusp_info *info)
{
	/*TODO: implement printing of kernel details here*/
	return;
}

static int check_swap_map_page(unsigned long offset)
{
	return !((offset -1) % PFNS_PER_PAGE);
}

static int swsusp_read(void)
{
	struct swsusp_info *info;
	int ret;
	unsigned long start_ms, temp, disk_read_ms = 0;
	unsigned long copy_page_ms = 0;
	unsigned long offset;
	unsigned long src_pfn, dst_pfn, bounce_pfn;
	unsigned int pending_pfns, nr_read_pages;
	unsigned long *pfns;
	unsigned long pfn_index = 0;
	unsigned long MBs, MBPS, DDR_MBPS;
	unsigned long read_size;
	unsigned long read_meta_pages, rem_meta_pages;
	int temp_read_pfns = 65536; //256 MB
	int Status;
	UINT64 temp_read_addr = 0x150000000;

	start_ms = GetTimerCountms();

	info = AllocatePages(1);
	if (!info) {
		printf("Failed to allocate memory for swsusp_info %d\n",__LINE__);
		return -1;
	}

	/* read swsusp_info struct at offset 2 */
	ret = read_image(SWAP_INFO_OFFSET, info, 1);
	if (ret) {
		printf("Failed to read swsusp_info %d\n", __LINE__);
		return -1;
	}

	resume_hdr = (struct arch_hibernate_hdr *)info;
	swsusp_info = info;
	nr_meta_pages = info->pages - info->image_pages - 1;
	nr_copy_pages = info->image_pages;
	printf("Total pages to copy = %lu Total meta pages = %lu\n", nr_copy_pages, nr_meta_pages);
	offset = SWAP_INFO_OFFSET + 1 ;

	/* Allocate memory for pfn indexes */
	pfns = AllocatePages(nr_meta_pages);
	if (!pfns) {
		printf("Failed to allocate memory for storing pfn meta data %d\n",
			__LINE__);
		return -1;
	}

	/* First swap_map page can have max (PFNS_PER_PAGE - 2) pfn indexes */
	read_size = nr_meta_pages < (PFNS_PER_PAGE - 2) ? nr_meta_pages : PFNS_PER_PAGE - 2;
	ret = read_image(offset, pfns, read_size);
	if (ret) {
		printf("Failed to read meta pages from disk %d\n", __LINE__);
		return -1;
	}
	offset += read_size;
	read_meta_pages = read_size;

	if (nr_meta_pages >= PFNS_PER_PAGE - 2) {
		/* skip swap_map page */
		offset++;
		while (nr_meta_pages != read_meta_pages) {
			rem_meta_pages = nr_meta_pages - read_meta_pages;
			read_size = rem_meta_pages > PFNS_PER_PAGE - 1 ? PFNS_PER_PAGE - 1 : rem_meta_pages;
			ret = read_image(offset, pfns + read_meta_pages * PFNS_PER_PAGE, read_size);
			if (ret) {
				printf("Failed to read meta pages from disk %d\n", __LINE__);
				return -1;
			}
			read_meta_pages += read_size;
			offset += read_size;

			/* skip swap_map page */
			if (read_size == PFNS_PER_PAGE - 1)
				offset++;
		}
	}

	if (read_meta_pages != nr_meta_pages) {
		printf("Mismatch in reading nr_meta_pages\n");
		return -1;
	}

	print_kernel_details(info);

	Status = gBS->AllocatePages (AllocateAddress, EfiBootServicesData,
					temp_read_pfns, &temp_read_addr);
	if (Status == EFI_SUCCESS) {
		printf("Temp read alloction at 0x%p - 0x%p\n", temp_read_addr,
				temp_read_addr + (temp_read_pfns << PAGE_SHIFT));
	} else {
		printf("Temp read alloc failed at 0x%p\n", temp_read_addr);
		return -1;
	}

	get_uefi_memory_map();

	pending_pfns = nr_copy_pages;
	printf("Reading pages:     ");
	while (pending_pfns > 0) {
		/* read pages in chunks to improve disk read performance */
		nr_read_pages = pending_pfns > temp_read_pfns ? temp_read_pfns : pending_pfns;
		temp = GetTimerCountms();
		ret = read_image(offset, (void *)temp_read_addr, nr_read_pages);
		disk_read_ms += (GetTimerCountms() - temp);
		if (ret < 0) {
			printf("Failed to read data pages from disc %d\n", __LINE__);
			break;
		}
		src_pfn = temp_read_addr >> PAGE_SHIFT;
		while (nr_read_pages > 0) {
			/* Skip swap_map pages */
			if (!check_swap_map_page(offset)) {
				dst_pfn = pfns[pfn_index++];
				pending_pfns--;
				bounce_pfn = (unsigned long)next_bounce_buf >> PAGE_SHIFT;
				temp = GetTimerCountms();
				ret = copy_page_to_dst(src_pfn, dst_pfn, bounce_pfn);
				copy_page_ms += (GetTimerCountms() - temp);
				if (ret == 1)
					update_bounce_entry(dst_pfn);
			}
			src_pfn++;
			nr_read_pages--;
			offset++;
		}
	}
	printf("Done. \n");
	if (ret < 0) {
		printf("error swsusp_read\n");
		return -1;
	}

	MBs = (nr_copy_pages*PAGE_SIZE)/(1024*1024);
	MBPS = (MBs*1000)/disk_read_ms;
	DDR_MBPS = (MBs*1000)/copy_page_ms;
	printf("Image size = %lu MBs\n", MBs);
	printf("Time loading image (excluding bounce buffers) = %lu msecs\n", (GetTimerCountms() - start_ms));
	printf("Time spend - disk IO = %lu msecs (BW = %llu MBps)\n", disk_read_ms, MBPS);
	printf("Time spend - DDR copy = %llu msecs (BW = %llu MBps)\n", copy_page_ms, DDR_MBPS);

	printf("Image restore Completed...\n");
	printf("Total bounced Pages = %d (%lu MBs)\n", bounced_pages, (bounced_pages*PAGE_SIZE)/(1024*1024));
	return 0;
}

static void copy_bounce_and_boot_kernel(UINT64 relocateAddress)
{
	int Status;
	unsigned long cpu_resume = (unsigned long )resume_hdr->phys_reenter_kernel;

	/* TODO:
	 * We are not relocating the jump routine for now so avoid this copy
	 * gBS->CopyMem ((VOID*)relocateAddress, (VOID*)&JumpToKernel, PAGE_SIZE);
	 */

	/*
	 * The restore routine "JumpToKernel" copies the bounced pages after iterating
	 * through the bounce entry table and passes control to hibernated kernel after
	 * calling PreparePlatformHarware
	 */

	printf("Disable UEFI Boot services\n");
	printf("Kernel entry point = 0x%lx\n", cpu_resume);

	/*Shut down UEFI boot services*/
	Status = ShutdownUefiBootServices ();
	if (EFI_ERROR (Status)) {
		DEBUG ((EFI_D_ERROR,
			"ERROR: Can not shutdown UEFI boot services. Status=0x%X\n",
			Status));
		return;
	}

	asm __volatile__ (
		"mov x18, %[table_base]\n"
		"mov x19, %[count]\n"
		"mov x20, %[bounce_base]\n"
		"mov x21, %[resume]\n"
		"mov x22, %[disable_cache]\n"
		"b JumpToKernel"
		:
		:[table_base] "r" (bounce_pfn_entry_table), [count] "r" (bounce_entry_count),
		[bounce_base] "r" (bounce_base_addr), [resume] "r" (cpu_resume),
		[disable_cache] "r" (PreparePlatformHardware)
		:"x18", "x19", "x20", "x21", "x22", "memory");
}

void BootIntoHibernationImage(void)
{
	int ret;
	UINT64 base_addr_bounce_entries;
	int Status, nr_pages;

	printf("===============================\n");
	printf("Entrying Hibernation restore\n");

	Status = gBS->AllocatePages (AllocateAddress, EfiBootServicesData,
					MAX_BOUNCE_PAGES, &bounce_base_addr);
	if (Status == EFI_SUCCESS) {
		next_bounce_buf = (void *)bounce_base_addr;
		printf("Bounce buffer Allocated at 0x%p - 0x%p\n",
			next_bounce_buf, next_bounce_buf + MAX_BOUNCE_PAGES*PAGE_SIZE);
	} else {
		printf("Bounce buffer Allocation failed at 0x%p\n", next_bounce_buf);
		return;
	}

	nr_pages = DIV_ROUND_UP((sizeof(struct bounce_pfn_entry) * MAX_BOUNCE_PAGES), PAGE_SIZE);
	base_addr_bounce_entries = bounce_base_addr + MAX_BOUNCE_PAGES * PAGE_SIZE;

	Status = gBS->AllocatePages (AllocateAddress, EfiBootServicesData,
				nr_pages, &base_addr_bounce_entries );
	if (Status == EFI_SUCCESS) {
		bounce_pfn_entry_table = (void *)base_addr_bounce_entries;
		next_bounce_pfn_entry = bounce_pfn_entry_table;
		printf("Bounce entries Allocated at 0x%p - 0x%p\n", bounce_pfn_entry_table,
					(void *)bounce_pfn_entry_table + nr_pages * PAGE_SIZE);
	} else {
		printf("Bounce entries Allocation failed at 0x%p\n", bounce_pfn_entry_table);
		gBS->FreePages(bounce_base_addr, MAX_BOUNCE_PAGES);
		return;
	}

	relocation_base_addr = base_addr_bounce_entries + nr_pages * PAGE_SIZE;
	/* Allocate a page for relocation code */
	Status = gBS->AllocatePages (AllocateAddress, EfiLoaderCode, 1, &relocation_base_addr);
	if (Status == EFI_SUCCESS) {
		printf("Relocation code Allocated at 0x%p - 0x%p\n", relocation_base_addr,
							relocation_base_addr + PAGE_SIZE);
	} else {
		printf("Relocation code Allocation failed at 0x%p\n", relocation_base_addr);
		gBS->FreePages(base_addr_bounce_entries, nr_pages);
		gBS->FreePages(bounce_base_addr, MAX_BOUNCE_PAGES);
		return;
	}

	swsusp_header = AllocatePages(1);
	if(!swsusp_header) {
		printf("AllocatePages failed\n");
		goto free_bounce;
	}

	ret = read_image(0, swsusp_header, 1);
	if (ret) {
		DEBUG((EFI_D_ERROR, "Failed to read image at offset 0\n"));
		goto read_image_error;
	}

	if(!memcmp(HIBERNATE_SIG, swsusp_header->sig, 10)) {
		DEBUG ((EFI_D_ERROR, "Signature found. Proceeing with disk read...\n"));
	} else {
		printf("Signature not found. Aborhing hibernation\n");
		goto read_image_error;
	}

	ret = swsusp_read();
	if (ret) {
		printf("Filed swsusp_read \n");
		goto read_image_error;
	}

	copy_bounce_and_boot_kernel(relocation_base_addr);
	/* We should not reach here */

read_image_error:
	FreePages(swsusp_header, 1);
free_bounce:
	gBS->FreePages(bounce_base_addr, MAX_BOUNCE_PAGES);
	gBS->FreePages(base_addr_bounce_entries, nr_pages);
	gBS->FreePages(relocation_base_addr, 1);
	return;
}

#endif
