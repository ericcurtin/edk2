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
#include "BootStats.h"
#include <Library/DxeServicesTableLib.h>
#include <VerifiedBoot.h>

#define BUG(fmt, ...) {\
		printf("Fatal error " fmt, ##__VA_ARGS__);\
		while(1);\
	}

#define ALIGN_1GB(address) address &= ~((1 << 30) - 1)
#define ALIGN_2MB(address) address &= ~((1 << 21) - 1)

/* Reserved some free memory for UEFI use */
#define RESERVE_FREE_SIZE	1024*1024*10
struct free_ranges {
	UINT64 start;
	UINT64 end;
};

/* Holds free memory ranges read from UEFI memory map */
static struct free_ranges free_range_buf[100];
static int free_range_count;

struct mapped_range {
        UINT64 start, end;
        struct mapped_range * next;
};

/* number of data pages to be copied from swap */
static unsigned int nr_copy_pages;
/* number of meta pages or pages which hold pfn indexes */
static unsigned int nr_meta_pages;
/* number of image kernel pages bounced due to conflict with UEFI */
static unsigned long bounced_pages;

static struct arch_hibernate_hdr *resume_hdr;

struct pfn_block {
	unsigned long base_pfn;
	int available_pfns;
};

struct kernel_pfn_iterator {
	unsigned long *pfn_array;
	int cur_index;
	int max_index;
	struct pfn_block cur_block;
};
static struct kernel_pfn_iterator kernel_pfn_iterator;

static struct swsusp_header *swsusp_header;
/*
 * Bounce Pages - During the copy of pages from snapshot image to
 * RAM, certain pages can conflicts with concurrently running UEFI/ABL
 * pages. These pages are copied temporarily to bounce pages. And
 * during later stage, upon exit from UEFI boot services, these
 * bounced pages are copied to their real destination. bounce_pfn_entry
 * is used to store the location of temporary bounce pages and their
 * real destination.
 */
struct bounce_pfn_entry {
	UINT64 dst_pfn;
	UINT64 src_pfn;
};

/*
 * Size of the buffer where disk IO is performed.If any kernel pages are
 * destined to be here, they will be bounced.
 */
#define	DISK_BUFFER_SIZE	64*1024*1024
#define	DISK_BUFFER_PAGES	(DISK_BUFFER_SIZE / PAGE_SIZE)

#define	OUT_OF_MEMORY	-1

#define	BOUNCE_TABLE_ENTRY_SIZE	sizeof(struct bounce_pfn_entry)
#define	ENTRIES_PER_TABLE	(PAGE_SIZE / BOUNCE_TABLE_ENTRY_SIZE) - 1

/*
 * Bounce Tables -  bounced pfn entries are stored in bounced tables.
 * Bounce tables are discontinuous pages linked by the last element
 * of the page. Bounced table are allocated using unused pfn allocator.
 *
 *       ---------          	      ---------
 * 0   | dst | src |	----->	0   | dst | src |
 * 1   | dst | src |	|	1   | dst | src |
 * 2   | dst | src |	|	2   | dst | src |
 * .   |	   |	|	.   |           |
 * .   |	   |	|	.   |           |
 * 256 | addr|     |-----	256 |addr |     |------>
 *       --------- 		      ---------
 */
struct bounce_table {
	struct bounce_pfn_entry bounce_entry[ENTRIES_PER_TABLE];
	unsigned long next_bounce_table;
	unsigned long padding;
};

struct bounce_table_iterator {
	struct bounce_table *first_table;
	struct bounce_table *cur_table;
	/* next available free table entry */
	int cur_index;
};
struct bounce_table_iterator table_iterator;

unsigned long relocateAddress;

#define PFN_INDEXES_PER_PAGE		512
/* Final entry is used to link swap_map pages together */
#define ENTRIES_PER_SWAPMAP_PAGE 	(PFN_INDEXES_PER_PAGE - 1)

#define SWAP_INFO_OFFSET        (swsusp_header->image + 1)
#define FIRST_PFN_INDEX_OFFSET	(SWAP_INFO_OFFSET + 1)

#define SWAP_PARTITION_NAME	L"swap_a"

/*
 * target_addr  : address where page allocation is needed
 *
 * return	: 1 if address falls in free range
 * 		  0 if address is not in free range
 */
static int CheckFreeRanges (UINT64 target_addr)
{
	int i = 0;
	while (i < free_range_count) {
		if (target_addr >= free_range_buf[i].start &&
			target_addr < free_range_buf[i].end)
		return 1;
		i++;
	}
	return 0;
}

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

static void init_kernel_pfn_iterator(unsigned long *array)
{
	struct kernel_pfn_iterator *iter = &kernel_pfn_iterator;
	iter->pfn_array = array;
	iter->max_index = nr_copy_pages;
}

static int find_next_available_block(struct kernel_pfn_iterator *iter)
{
	int available_pfns;

	do {
		unsigned long cur_pfn, next_pfn;
		iter->cur_index++;
		if (iter->cur_index >= iter->max_index)
			BUG("index maxed out. Line %d\n", __LINE__);
		cur_pfn = iter->pfn_array[iter->cur_index];
		next_pfn = iter->pfn_array[iter->cur_index + 1];
		available_pfns = next_pfn - cur_pfn - 1;
	 } while (!available_pfns);

	iter->cur_block.base_pfn = iter->pfn_array[iter->cur_index];
	iter->cur_block.available_pfns = available_pfns;
	return 0;
}

static unsigned long get_unused_kernel_pfn(void)
{
	struct kernel_pfn_iterator *iter = &kernel_pfn_iterator;

	if (!iter->cur_block.available_pfns)
		find_next_available_block(iter);

	iter->cur_block.available_pfns--;
	return ++iter->cur_block.base_pfn;
}

/*
 * get a pfn which is unused by kernel and UEFI.
 *
 * unused pfns are pnfs which doesn't overlap with image kernel pages
 * or UEFI pages. These pfns are used for bounce pages, bounce tables
 * and relocation code.
 */
static unsigned long get_unused_pfn()
{
	unsigned long pfn;

	do {
		pfn = get_unused_kernel_pfn();
	} while(!CheckFreeRanges(pfn << PAGE_SHIFT));

	return pfn;
}

/*
 * Preallocation is done for performance reason. We want to map memory
 * as big as possible. So that UEFI can create bigger page table mappings.
 * We have seen mapping single page is taking time in terms of few ms.
 * But we cannot preallocate every free page, becasue that causes allocation
 * failures for UEFI. Hence allocate most of the free pages but some(10MB)
 * are kept unallocated for UEFI to use. If kernel has any destined pages in
 * this region, that will be bounced.
 */
static void preallocate_free_ranges(void)
{
	int i = 0, ret;
	int reservation_done = 0;
	UINT64 alloc_addr, range_size;
	UINT64 num_pages;

	for (i = free_range_count - 1; i >= 0 ; i--) {
		range_size = free_range_buf[i].end - free_range_buf[i].start;
		if (!reservation_done && range_size > RESERVE_FREE_SIZE) {
			/*
			 * We have more buffer. Remove reserved buf and allocate
			 * rest in the range.
			 */
			reservation_done = 1;
			alloc_addr = free_range_buf[i].start + RESERVE_FREE_SIZE;
			range_size -=  RESERVE_FREE_SIZE;
			num_pages = range_size/PAGE_SIZE;
			printf("Reserved range = 0x%lx - 0x%lx\n", free_range_buf[i].start,
								alloc_addr - 1);
			/* Modify the free range start */
			free_range_buf[i].start = alloc_addr;
		} else {
			alloc_addr = free_range_buf[i].start;
			num_pages = range_size/PAGE_SIZE;
		}

		ret = gBS->AllocatePages(AllocateAddress, EfiBootServicesData,
				num_pages, &alloc_addr);
		if(ret)
			printf("WARN: Prealloc falied LINE %d alloc_addr = 0x%lx\n",
							__LINE__, alloc_addr);
	}
}

/* Assumption: There is no overlap in the regions */
static struct mapped_range * add_range_sorted(struct mapped_range * head, UINT64 start, UINT64 end)
{
	struct mapped_range * elem, * p;
	EFI_STATUS status;

	status = gBS->AllocatePool(EfiBootServicesData, sizeof(struct mapped_range), (VOID *)&elem);
	if (status != EFI_SUCCESS) {
		printf("Failed to AllocatePool %d\n", __LINE__);
		return NULL;
	}
	elem->start = start;
	elem->end = end;
	elem->next = NULL;

	if (head == NULL)
		return elem;

	if (start <= head->start) {
		elem->next = head;
		return elem;
	}

	p = head;
	while (p->next != NULL && p->next->start < start)
		p = p->next;

	elem->next = p->next;
	p->next = elem;

	return head;
}

/*
 * Get the UEFI memory map to collect ranges of
 * memory of type EfiConventional
 */
static int get_conventional_memory_ranges(void)
{
	EFI_MEMORY_DESCRIPTOR	*MemMap;
	EFI_MEMORY_DESCRIPTOR	*MemMapPtr;
	UINTN			MemMapSize;
	UINTN			MapKey, DescriptorSize;
	UINTN			Index;
	UINT32			DescriptorVersion;
	EFI_STATUS		Status;
	int index = 0;

	MemMapSize = 0;
	MemMap     = NULL;

	Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
				&DescriptorSize, &DescriptorVersion);
	if (Status != EFI_BUFFER_TOO_SMALL) {
		printf("ERROR: Undefined response get memory map\n");
		return -1;
	}
	if (CHECK_ADD64 (MemMapSize, EFI_PAGE_SIZE)) {
		printf("ERROR: integer Overflow while adding additional"
					"memory to MemMapSize");
		return -1;
	}
	MemMapSize = MemMapSize + EFI_PAGE_SIZE;
	MemMap = AllocateZeroPool (MemMapSize);
	if (!MemMap) {
		printf("ERROR: Failed to allocate memory for memory map\n");
		return -1;
	}
	MemMapPtr = MemMap;
	Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
				&DescriptorSize, &DescriptorVersion);
	if (EFI_ERROR (Status)) {
		printf("ERROR: Failed to query memory map\n");
		FreePool (MemMapPtr);
		return -1;
	}
	for (Index = 0; Index < MemMapSize / DescriptorSize; Index ++) {
		if (MemMap->Type == EfiConventionalMemory) {
			free_range_buf[index].start = MemMap->PhysicalStart;
			free_range_buf[index].end =  MemMap->PhysicalStart + MemMap->NumberOfPages * PAGE_SIZE;
			printf("Free Range 0x%lx --- 0x%lx\n",free_range_buf[index].start,
					free_range_buf[index].end);
			index++;
		}
		MemMap = (EFI_MEMORY_DESCRIPTOR *)((UINTN)MemMap + DescriptorSize);
	}
	free_range_count = index;
	FreePool (MemMapPtr);
	return 0;
}

struct partition_details {
	EFI_BLOCK_IO_PROTOCOL *BlockIo;
	EFI_HANDLE *Handle;
	int blocksPerPage;
};
static struct partition_details swap_details;

static int verify_swap_partition(void)
{
	int Status;
	EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
	EFI_HANDLE *Handle = NULL;

	Status = PartitionGetInfo (SWAP_PARTITION_NAME, &BlockIo, &Handle);
	if (Status != EFI_SUCCESS)
		return Status;

	if (!Handle) {
		printf("EFI handle for swap partition is corrupted\n");
		return -1;
	}

	if (CHECK_ADD64 (BlockIo->Media->LastBlock, 1)) {
		printf("Integer overflow while adding LastBlock and 1\n");
		return -1;
	}

	if ((MAX_UINT64 / (BlockIo->Media->LastBlock + 1)) <
			(UINT64)BlockIo->Media->BlockSize) {
		printf("Integer overflow while multiplying LastBlock and BlockSize\n");
		return -1;
	}

	swap_details.BlockIo = BlockIo;
	swap_details.Handle = Handle;
	swap_details.blocksPerPage = EFI_PAGE_SIZE / BlockIo->Media->BlockSize;
	return 0;
}

static int read_image(unsigned long offset, VOID *Buff, int nr_pages)
{
	int Status;
	EFI_BLOCK_IO_PROTOCOL *BlockIo = swap_details.BlockIo;
	EFI_LBA Lba;

	Lba = offset * swap_details.blocksPerPage;
	Status = BlockIo->ReadBlocks (BlockIo,
			BlockIo->Media->MediaId,
			Lba,
			EFI_PAGE_SIZE * nr_pages,
			(VOID*)Buff);
	if (Status != EFI_SUCCESS) {
		printf("Read image failed Line = %d\n", __LINE__);
		return Status;
	}

	return 0;
}

static int is_current_table_full(struct bounce_table_iterator *bti)
{
	return (bti->cur_index == ENTRIES_PER_TABLE);
}

static void alloc_next_table(struct bounce_table_iterator *bti)
{
	/* Allocate and chain next bounce table */
	bti->cur_table->next_bounce_table = get_unused_pfn() << PAGE_SHIFT;
	bti->cur_table = (struct bounce_table *) bti->cur_table->next_bounce_table;
	bti->cur_index = 0;
}

static struct bounce_pfn_entry * find_next_bounce_entry(void)
{
	struct bounce_table_iterator *bti = &table_iterator;

	if (is_current_table_full(bti))
		alloc_next_table(bti);

	return &bti->cur_table->bounce_entry[bti->cur_index++];
}

static void update_bounce_entry(UINT64 dst_pfn, UINT64 src_pfn)
{
	struct bounce_pfn_entry *entry;

	entry = find_next_bounce_entry();
	entry->dst_pfn = dst_pfn;
	entry->src_pfn = src_pfn;
	bounced_pages++;
}

/*
 * Copy page to destination if page is free and is not in reserved area.
 * Bounce page otherwise.
 */
static void copy_page_to_dst(unsigned long src_pfn, unsigned long dst_pfn)
{
	UINT64 target_addr = dst_pfn << PAGE_SHIFT;

	if (CheckFreeRanges(target_addr)) {
		copyPage(src_pfn, dst_pfn);
	} else {
		unsigned long bounce_pfn = get_unused_pfn();
		copyPage(src_pfn, bounce_pfn);
		update_bounce_entry(dst_pfn, bounce_pfn);
	}
}

static void print_image_kernel_details(struct swsusp_info *info)
{
	/*TODO: implement printing of kernel details here*/
	return;
}

/*
 * swsusp_header->image points to first swap_map page. From there onwards,
 * swap_map pages are repeated at every PFN_INDEXES_PER_PAGE intervals.
 * This function returns true if offset belongs to a swap_map page.
 */
static int check_swap_map_page(unsigned long offset)
{
	offset -= swsusp_header->image;
	return (offset % PFN_INDEXES_PER_PAGE) == 0;
}

static int read_swap_info_struct(void)
{
	struct swsusp_info *info;

	BootStatsSetTimeStamp (BS_KERNEL_LOAD_START);

	info = AllocatePages(1);
	if (!info) {
		printf("Memory alloc failed Line %d\n",__LINE__);
		return -1;
	}

	if (read_image(SWAP_INFO_OFFSET, info, 1)) {
		printf("Failed to read Line %d\n", __LINE__);
		FreePages(info, 1);
		return -1;
	}

	resume_hdr = (struct arch_hibernate_hdr *)info;
	nr_meta_pages = info->pages - info->image_pages - 1;
	nr_copy_pages = info->image_pages;
	printf("Total pages to copy = %lu Total meta pages = %lu\n",
				nr_copy_pages, nr_meta_pages);
	print_image_kernel_details(info);
	return 0;
}

/*
 * Reads image kernel pfn indexes by stripping off interleaved swap_map pages.
 *
 * swap_map pages are particularly useful when swap slot allocations are
 * randomized. For bootloader based hibernation we have disabled this for
 * performance reasons. But swap_map pages are still interleaved because
 * kernel/power/snapshot.c is written to handle both scenarios(sequential
 * and randomized swap slot).
 *
 * Snapshot layout in disk with randomization disabled for swap allocations in
 * kernel looks likes:
 *
 *			disk offsets
 *				|
 *				|
 * 				V
 * 				   -----------------------
 * 				0 |     header		  |
 * 				  |-----------------------|
 * 				1 |  swap_map page 0	  |
 * 				  |-----------------------|	      ------
 * 				2 |  swsusp_info struct	  |		 ^
 * 	------			  |-----------------------|		 |
 * 	  ^			3 |  PFN INDEX Page 0	  |		 |
 * 	  |		          |-----------------------|		 |
 * 	  |	 		4 |  PFN INDEX Page 1	  |	      	 |
 *   	  |			  |-----------------------|	511 swap map entries
 * 510 pfn index pages		  |     :       :         |		 |
 *  	  |			  |  	:	:	  |		 |
 *  	  |			  |  	:	:	  |		 |
 *  	  |			  |-----------------------|		 |
 *  	  V		      512 |  PFN INDEX Page 509	  |		 V
 * 	------	    	          |-----------------------|	       -----
 * 		    	      513 |  swap_map page 1  	  |
 * 	------			  |-----------------------|	       ------
 * 	  ^		      514 |  PFN INDEX Page 510   |		 ^
 * 	  |		          |-----------------------|		 |
 * 	  |	 	      515 |  PFN INDEX Page 511	  |		 |
 *   	  |			  |-----------------------|		 |
 * 511 pfn index pages		  |     :       :         |	511 swap map entries
 *  	  |			  |  	:	:	  |		 |
 *  	  |			  |  	:	:	  |		 |
 *  	  |			  |-----------------------|		 |
 *  	  V		     1024 |  PFN INDEX Page 1021  |		 V
 * 	------	    	          |-----------------------|	       ------
 * 		    	     1025 |  swap_map page 2  	  |
 * 	------			  |-----------------------|
 * 	  ^		     1026 |  PFN INDEX Page 1022  |
 * 	  |		          |-----------------------|
 * 	  |	 	     1027 |  PFN INDEX Page 1023  |
 *   	  |			  |-----------------------|
 * 511 pfn index pages		  |     :       :         |
 *  	  |			  |  	:	:	  |
 *  	  |			  |  	:	:	  |
 *  	  |			  |-----------------------|
 *  	  V		     1536 |  PFN INDEX Page 1532  |
 * 	------	    	          |-----------------------|
 * 		    	     1537 |  swap_map page 3  	  |
 * 				  |-----------------------|
 * 			     1538 |  PFN INDEX Page 1533  |
 * 			          |-----------------------|
 * 			     1539 |  PFN INDEX Page 1534  |
 * 				  |-----------------------|
 * 				  |     :       :         |
 * 				  |  	:	:	  |
 */
static unsigned long* read_kernel_image_pfn_indexes(unsigned long *offset)
{
	unsigned long *pfn_array, *array_index;
	unsigned long pending_pages = nr_meta_pages;
	unsigned long pages_to_read, pages_read = 0;
	unsigned long disk_offset;
	int loop = 0, ret;

	pfn_array = AllocatePages(nr_meta_pages);
	if (!pfn_array) {
		printf("Memory alloc failed Line %d\n", __LINE__);
		return NULL;
	}

	disk_offset = FIRST_PFN_INDEX_OFFSET;
	/*
	 * First swap_map page has one less pfn_index page
	 * because of presence of swsusp_info struct. Handle
	 * it separately.
	 */
	pages_to_read = MIN(pending_pages, ENTRIES_PER_SWAPMAP_PAGE - 1);
	array_index = pfn_array;
	do {
		ret = read_image(disk_offset, array_index, pages_to_read);
		if (ret) {
			printf("Disk read failed Line %d\n", __LINE__);
			goto err;
		}
		pages_read += pages_to_read;
		pending_pages -= pages_to_read;
		if (!pending_pages)
			break;
		loop++;
		/*
		 * swsusp_header->image points to first swap_map page. From there onwards,
		 * swap_map pages are repeated at PFN_INDEXES_PER_PAGE interval.
		 * pfn_index pages follows the swap map page. So we can arrive at
		 * next pfn_index by using below formula,
		 *
		 * base_swap_map_slot + PFN_INDEXES_PER_PAGE * n + 1
		 */
		disk_offset = swsusp_header->image + (PFN_INDEXES_PER_PAGE * loop) + 1;
		pages_to_read = MIN(pending_pages, ENTRIES_PER_SWAPMAP_PAGE);
		array_index = pfn_array + pages_read * PFN_INDEXES_PER_PAGE;
	} while (1);

	*offset = disk_offset + pages_to_read;
	return pfn_array;
err:
	FreePages(pfn_array, nr_meta_pages);
	return NULL;
}

static int read_data_pages(unsigned long *kernel_pfn_indexes,
			unsigned long offset, void *disk_read_buffer)
{
	unsigned int pending_pages, nr_read_pages;
	unsigned long temp, disk_read_ms = 0;
	unsigned long copy_page_ms = 0;
	unsigned long src_pfn, dst_pfn;
	unsigned long pfn_index = 0;
	unsigned long MBs, MBPS, DDR_MBPS;
	int ret;

	pending_pages = nr_copy_pages;
	while (pending_pages > 0) {
		/* read pages in chunks to improve disk read performance */
		nr_read_pages = pending_pages > DISK_BUFFER_PAGES ? DISK_BUFFER_PAGES : pending_pages;
		temp = GetTimerCountms();
		ret = read_image(offset, disk_read_buffer, nr_read_pages);
		disk_read_ms += (GetTimerCountms() - temp);
		if (ret < 0) {
			printf("Disk read failed Line %d\n", __LINE__);
			return -1;
		}
		src_pfn = (unsigned long) disk_read_buffer >> PAGE_SHIFT;
		while (nr_read_pages > 0) {
			/* skip swap_map pages */
			if (!check_swap_map_page(offset)) {
				dst_pfn = kernel_pfn_indexes[pfn_index++];
				pending_pages--;
				temp = GetTimerCountms();
				copy_page_to_dst(src_pfn, dst_pfn);
				copy_page_ms += (GetTimerCountms() - temp);
			}
			src_pfn++;
			nr_read_pages--;
			offset++;
		}
	}
	BootStatsSetTimeStamp (BS_KERNEL_LOAD_DONE);

	MBs = (nr_copy_pages*PAGE_SIZE)/(1024*1024);
	if (disk_read_ms == 0 || copy_page_ms == 0)
		return 0;

	MBPS = (MBs*1000)/disk_read_ms;
	DDR_MBPS = (MBs*1000)/copy_page_ms;

	printf("Image size = %lu MBs\n", MBs);
	printf("Time spend - disk IO = %lu msecs (BW = %llu MBps)\n", disk_read_ms, MBPS);
	printf("Time spend - DDR copy = %llu msecs (BW = %llu MBps)\n", copy_page_ms, DDR_MBPS);

	return 0;
}

static struct mapped_range * get_uefi_sorted_memory_map()
{
	EFI_MEMORY_DESCRIPTOR	*MemMap;
	EFI_MEMORY_DESCRIPTOR	*MemMapPtr;
	UINTN			MemMapSize;
	UINTN			MapKey, DescriptorSize;
	UINTN			Index;
	UINT32			DescriptorVersion;
	EFI_STATUS		Status;

	struct mapped_range * uefi_map = NULL;
	MemMapSize = 0;
	MemMap     = NULL;

	Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
				&DescriptorSize, &DescriptorVersion);
	if (Status != EFI_BUFFER_TOO_SMALL) {
		printf("ERROR: Undefined response get memory map\n");
		return NULL;
	}
	if (CHECK_ADD64 (MemMapSize, EFI_PAGE_SIZE)) {
		printf("ERROR: integer Overflow while adding additional"
					"memory to MemMapSize");
		return NULL;
	}
	MemMapSize = MemMapSize + EFI_PAGE_SIZE;
	MemMap = AllocateZeroPool (MemMapSize);
	if (!MemMap) {
		printf("ERROR: Failed to allocate memory for memory map\n");
		return NULL;
	}
	MemMapPtr = MemMap;
	Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
				&DescriptorSize, &DescriptorVersion);
	if (EFI_ERROR (Status)) {
		printf("ERROR: Failed to query memory map\n");
		FreePool (MemMapPtr);
		return NULL;
	}
	for (Index = 0; Index < MemMapSize / DescriptorSize; Index ++) {
		uefi_map = add_range_sorted(uefi_map,
			MemMap->PhysicalStart,
			MemMap->PhysicalStart + MemMap->NumberOfPages * PAGE_SIZE);

		if (!uefi_map) {
			printf("ERROR: uefi_map is NULL\n");
			return NULL;
		}

		MemMap = (EFI_MEMORY_DESCRIPTOR *)((UINTN)MemMap + DescriptorSize);
	}
	FreePool (MemMapPtr);
	return uefi_map;
}

static EFI_STATUS create_mapping(UINTN addr, UINTN size)
{
	EFI_STATUS status;
	EFI_GCD_MEMORY_SPACE_DESCRIPTOR Descriptor;
	printf("Address: %llx Size: %llx\n", addr, size);

	status = gDS->GetMemorySpaceDescriptor(addr, &Descriptor);
	if (EFI_ERROR(status)) {
		printf("Failed getMemorySpaceDescriptor Line %d\n", __LINE__);
		return status;
	}

	if (Descriptor.GcdMemoryType != EfiGcdMemoryTypeMemoryMappedIo) {
		if (Descriptor.GcdMemoryType != EfiGcdMemoryTypeNonExistent){
			status = gDS->RemoveMemorySpace(addr, size);
			printf("Falied RemoveMemorySpace %d: %d\n", __LINE__, status);
		}
		status = gDS->AddMemorySpace(EfiGcdMemoryTypeReserved, addr, size, EFI_MEMORY_UC);
		if (EFI_ERROR(status)) {
			printf("Failed to AddMemorySpace 0x%x, size 0x%x\n", addr, size);
			return status;
		}

		status = gDS->SetMemorySpaceAttributes (addr, size, EFI_MEMORY_UC);
		if (EFI_ERROR(status)) {
			printf("Failed to SetMemorySpaceAttributes 0x%x, size 0x%x\n", addr, size);
				return status;
		}
	}

	return EFI_SUCCESS;
}

/*
 * Determine the unmapped uefi memory from the list 'uefi_mapped_sorted_list'
 * and map all the unmapped regions.
 */
static EFI_STATUS uefi_map_unmapped()
{
	struct mapped_range * uefi_mapped_sorted_list, * cur, * next;
	EFI_STATUS Status;

	uefi_mapped_sorted_list = get_uefi_sorted_memory_map();
	if (!uefi_mapped_sorted_list) {
		printf("ERROR: Unable to get UEFI memory map\n");
		return -1;
	}

	cur = uefi_mapped_sorted_list;
	next = cur->next;

	while (cur) {
		if (next && (next->start > cur->end)) {
			Status = create_mapping(cur->end, next->start - cur->end);
			if (Status != EFI_SUCCESS) {
				printf("ERROR: Mapping failed\n");
				return Status;
			}
		}
		Status = gBS->FreePool(cur);
		if(Status != EFI_SUCCESS) {
			printf("FreePool failed %d\n", __LINE__);
			return -1;
		}
		cur = next;
		if (next)
			next = next->next;
	}

	return EFI_SUCCESS;
}

#define PT_ENTRIES_PER_LEVEL 512

static void set_rw_perm(unsigned long *entry)
{
	/* Clear AP perm bits */
	*entry &= ~(0x3UL << 6);
}

static void set_ex_perm(unsigned long *entry)
{
	/* Clear UXN and PXN bits */
	*entry &= ~(0x3UL << 53);
}

static int relocate_pagetables(int level, unsigned long *entry, int pt_count)
{
	int i;
	unsigned long mask;
	unsigned long *page_addr;
	unsigned long apPerm;

	apPerm = *entry & (0x3 << 6);
	apPerm = apPerm >> 6;
	/* Strip out lower and higher page attribute fields */
	mask = ~(0xFFFFUL << 48 | 0XFFFUL);

	/* Invalid entry */
	if (level > 3 || !(*entry & 0x1))
		return pt_count;

	if (level == 3 ) {
		if ((*entry & mask) == relocateAddress)
			set_ex_perm(entry);
		if (apPerm == 2 || apPerm == 3)
			set_rw_perm(entry);
		return pt_count;
	}

	/* block entries */
	if ((*entry & 0x3) == 1) {
		unsigned long addr = relocateAddress;
		if(level == 1)
			ALIGN_1GB(addr);
		if (level == 2)
			ALIGN_2MB(addr);
		if ((*entry & mask) == addr)
			set_ex_perm(entry);
		if(apPerm == 2 || apPerm == 3)
			set_rw_perm(entry);
		return pt_count;
	}

	/* Control reaches here only if it is a table entry */

	page_addr = (unsigned long*)(get_unused_pfn() << PAGE_SHIFT);

	gBS->CopyMem ((void *)(page_addr), (void *)(*entry & mask), PAGE_SIZE);
	pt_count++;
	/* Clear off the old address alone */
	*entry &= ~mask;
	/* Fill new table address */
	*entry |= (unsigned long )page_addr;

	for (i = 0 ; i < PT_ENTRIES_PER_LEVEL; i++)
		pt_count = relocate_pagetables(level + 1, page_addr + i, pt_count);

	return pt_count;
}

static unsigned long get_ttbr0()
{
	unsigned long base;

	asm __volatile__ (
	"mrs %[ttbr0_base], ttbr0_el1\n"
	:[ttbr0_base] "=r" (base)
	:
	:"memory");

	return base;
}

static unsigned long copy_page_tables()
{
	unsigned long old_ttbr0 = get_ttbr0();
	unsigned long new_ttbr0;
	int pt_count = 0;

	new_ttbr0 = get_unused_pfn() << PAGE_SHIFT;
	gBS->CopyMem ((void *)(new_ttbr0), (void *)(old_ttbr0), PAGE_SIZE);
	pt_count = relocate_pagetables(0, (unsigned long *)new_ttbr0, 1);

	printf("Copied %d Page Tables\n", pt_count);
	return new_ttbr0;
}

static int restore_snapshot_image(void)
{
	int ret;
	void *disk_read_buffer;
	unsigned long start_ms, offset;
	unsigned long *kernel_pfn_indexes;
	struct bounce_table_iterator *bti = &table_iterator;

	start_ms = GetTimerCountms();
	ret = read_swap_info_struct();
	if (ret < 0)
		return ret;

	kernel_pfn_indexes = read_kernel_image_pfn_indexes(&offset);
	if (!kernel_pfn_indexes)
		return -1;
	init_kernel_pfn_iterator(kernel_pfn_indexes);

	disk_read_buffer =  AllocatePages(DISK_BUFFER_PAGES);
	if (!disk_read_buffer) {
		printf("Memory alloc failed Line %d\n", __LINE__);
		return -1;
	} else {
		printf("Disk buffer alloction at 0x%p - 0x%p\n", disk_read_buffer,
				disk_read_buffer + DISK_BUFFER_SIZE - 1);
	}

	printf("Mapping Regions:\n");
	ret = uefi_map_unmapped();
	if (ret < 0) {
		printf("Error mapping unmapped regions\n");
		goto err;
	}

	/*
	 * No dynamic allocation beyond this point. If not honored it will
	 * result in corruption of pages.
	 */
	get_conventional_memory_ranges();
	preallocate_free_ranges();

	bti->first_table = (struct bounce_table *)(get_unused_pfn() << PAGE_SHIFT);
	bti->cur_table = bti->first_table;

	ret = read_data_pages(kernel_pfn_indexes, offset, disk_read_buffer);
	if (ret < 0) {
		printf("error in restore_snapshot_image\n");
		goto err;
	}

	printf("Time loading image (excluding bounce buffers) = %lu msecs\n", (GetTimerCountms() - start_ms));
	printf("Image restore Completed...\n");
	printf("Total bounced Pages = %d (%lu MBs)\n", bounced_pages, (bounced_pages*PAGE_SIZE)/(1024*1024));
err:
	FreePages(disk_read_buffer, DISK_BUFFER_PAGES);
	return ret;
}

static void copy_bounce_and_boot_kernel()
{
	int Status;
	struct bounce_table_iterator *bti = &table_iterator;
	unsigned long cpu_resume = (unsigned long )resume_hdr->phys_reenter_kernel;
	unsigned long ttbr0;

	/*
	 * The restore routine "JumpToKernel" copies the bounced pages after iterating
	 * through the bounce entry table and passes control to hibernated kernel after
	 * calling _PreparePlatformHarware
	 *
	 * Disclaimer: JumpToKernel.s is less than PAGE_SIZE
	 */
	gBS->CopyMem ((VOID*)relocateAddress, (VOID*)&JumpToKernel, PAGE_SIZE);
	ttbr0 = copy_page_tables();

	printf("Disable UEFI Boot services\n");
	printf("Kernel entry point = 0x%lx\n", cpu_resume);
	printf("Relocation code at = 0x%lx\n", relocateAddress);

	BootStatsSetTimeStamp (BS_BL_END);

	/* Shut down UEFI boot services */
	Status = ShutdownUefiBootServices ();
	if (EFI_ERROR (Status)) {
		DEBUG ((EFI_D_ERROR,
			"ERROR: Can not shutdown UEFI boot services."
			" Status=0x%X\n", Status));
		return;
	}

	asm __volatile__ (
		"mov	x18, %[ttbr_reg]\n"
		"msr 	ttbr0_el1, x18\n"
		"dsb	sy\n"
		"isb\n"
		"ic	iallu\n"
		"dsb	sy\n"
		"isb\n"
		"tlbi	vmalle1\n"
		"dsb	sy\n"
		"isb\n"
		:
		:[ttbr_reg] "r" (ttbr0)
		:"x18", "memory");

	asm __volatile__ (
		"mov x18, %[table_base]\n"
		"mov x19, %[count]\n"
		"mov x21, %[resume]\n"
		"mov x22, %[relocate_code]\n"
		"br x22"
		:
		:[table_base] "r" (bti->first_table),
		[count] "r" (bounced_pages),
		[resume] "r" (cpu_resume),
		[relocate_code] "r" (relocateAddress)
		:"x18", "x19", "x21", "x22", "memory");
}

static int check_for_valid_header(void)
{
	swsusp_header = AllocatePages(1);
	if (!swsusp_header) {
		printf("Memory alloc failed Line %d\n", __LINE__);
		return -1;
	}

	if (verify_swap_partition()) {
		printf("Failled verify_swap_partition\n");
		goto read_image_error;
	}

	if (read_image(0, swsusp_header, 1)) {
		printf("Disk read failed Line %d\n", __LINE__);
		goto read_image_error;
	}

	if (memcmp(HIBERNATE_SIG, swsusp_header->sig, 10)) {
		printf("Signature not found. Aborting hibernation\n");
		goto read_image_error;
	}

	printf("Image slot at 0x%lx\n", swsusp_header->image);
	if (swsusp_header->image != 1) {
		printf("Invalid swap slot. Aborting hibernation!");
		goto read_image_error;
	}

	printf("Signature found. Proceeding with disk read...\n");
	return 0;

read_image_error:
	FreePages(swsusp_header, 1);
	return -1;
}

static void erase_swap_signature(void)
{
	int status;
	EFI_BLOCK_IO_PROTOCOL *BlockIo = swap_details.BlockIo;

	swsusp_header->sig[0] = ' ';
	status = BlockIo->WriteBlocks (BlockIo, BlockIo->Media->MediaId, 0,
			BlockIo->Media->BlockSize, (VOID*)swsusp_header);
	if (status != EFI_SUCCESS)
		printf("Failed to erase swap signature\n");
}

void BootIntoHibernationImage(BootInfo *Info)
{
	int ret;
	EFI_STATUS Status = EFI_SUCCESS;

	printf("===============================\n");
	printf("Entrying Hibernation restore\n");

	if (check_for_valid_header() < 0)
		goto err;

	Status = LoadImageAndAuth (Info, TRUE);
	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "Failed to set ROT and Bootstate : %r\n", Status));
		goto err;
	}

	ret = restore_snapshot_image();
	if (ret) {
		printf("Failed restore_snapshot_image \n");
		goto err;
	}

	relocateAddress = get_unused_pfn() << PAGE_SHIFT;

	/* Reset swap signature now */
	erase_swap_signature();
	copy_bounce_and_boot_kernel();
	/* Control should not reach here */

err:	/*
	 * Erase swap signature to avoid kernel restoring the
	 * hibernation image
	 */
	erase_swap_signature();
	return;
}
#endif
