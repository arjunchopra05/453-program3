#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/statvfs.h>

#include "cpe453fs.h"

struct Args
{
	int fd;
};

// --------------------Read-write------------------------------------

/* helper functions for read-write section only */
#define A_s 4
#define M_s 2
#define C_s 1
#define AMC_s 7
#define MC_s 3
void update_tim(unsigned char *block_buf, uint8_t field) {
	printf("HELPER: update time\n");
	struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    uint32_t sec = (uint32_t)now.tv_sec;
    uint32_t nsec = (uint32_t)now.tv_nsec;
	if (field & A_s) {
		memcpy(&block_buf[24], &sec, 4);
		memcpy(&block_buf[28], &nsec, 4);
	}
	if (field & M_s) {
		memcpy(&block_buf[32], &sec, 4);
		memcpy(&block_buf[36], &nsec, 4);
	}
	if (field & C_s) {
		memcpy(&block_buf[40], &sec, 4);
		memcpy(&block_buf[44], &nsec, 4);
	}
}

uint32_t allocate_block(void *args) {
	printf("HELPER: allocate block\n");
    struct Args *fs = (struct Args*)args;
	uint32_t new_bnum;
	uint32_t free_head = 0;
	uint32_t free_second;

	//get free list head
	unsigned char super[4096];
	readblock(fs->fd, super, 0);
	memcpy(&free_head, &super[4092], 4);

	if (free_head) {
		//get next in free list
		unsigned char free_head_buf[4096];
		readblock(fs->fd, free_head_buf, free_head);
		memcpy(&free_second, &free_head_buf[4], 4);
		//write second free block to free list head in super block
		memcpy(&super[4092], &free_second, 4);
		writeblock(fs->fd, super, 0);
		new_bnum = free_head;
	}
	else { //allocate new block by growing size of file by 1 block's worth of bytes
		struct stat st_buf;
		if (fstat(fs->fd, &st_buf) < 0) {
			perror("allocate_block(): fstat failed");
			return 0;
		}
		off_t size = st_buf.st_size;
		new_bnum = (uint32_t) size / 4096;
		if (ftruncate(fs->fd, size + 4096) < 0) {
            perror("allocate_block(): ftruncate failed");
            return 0;
        }
		unsigned char empty_buf[4096];
		memset(empty_buf, 0, sizeof(empty_buf));
		writeblock(fs->fd, empty_buf, new_bnum);
	}
    return new_bnum;
}

void free_block(void *args, uint32_t block_num) {
	printf("HELPER: free_block\n");
    struct Args *fs = (struct Args*)args;

	unsigned char super[4096];
	readblock(fs->fd, super, 0);
	uint32_t free_head;
	memcpy(&free_head, &super[4092], 4);

	uint32_t bnum = block_num;
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t next_extents;
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);

	if (type_code == 1 || type_code == 5) return;
	
	while (bnum) {
		type_code = 5;
		memcpy(&next_extents, &block_buf[4092], 4);
		memcpy(&block_buf[0], &type_code, 4);
		memcpy(&block_buf[4092], &free_head, 4);
		writeblock(fs->fd, block_buf, bnum);
		free_head = bnum;
		bnum = next_extents;
	}
	memcpy(&super[4092], &free_head, 4);
	writeblock(fs->fd, super, 0);
}

int add_dir_entry(void *args, uint32_t parent_dir, uint16_t len, uint32_t ino_map, const char *name) {
	printf("HELPER: add directory entry\n");
    struct Args *fs = (struct Args*)args;
	uint32_t dir_num = parent_dir;
	unsigned char dir_buf[4096];

	unsigned char ino_buf[4096];
	readblock(fs->fd, ino_buf, parent_dir);
	
	uint32_t type_code;
	memcpy(&type_code, &ino_buf[0], 4);
	uint16_t mode;
	memcpy(&mode, &ino_buf[4], 2);
	if (type_code != 2) return -EINVAL;
	else if ((mode & S_IFDIR) == 0) return -EINVAL;
	
	uint32_t next_extents;
	int entry_base;
	uint16_t entry_len;
	while (dir_num) {
		readblock(fs->fd, dir_buf, dir_num);

		memcpy(&type_code, &dir_buf[0], 4);
		if (type_code == 2) entry_base = 64;
		else if (type_code == 3) entry_base = 4;
		else return -EINVAL;

		memcpy(&next_extents, &dir_buf[4092], 4);
		if (next_extents) {
			dir_num = next_extents;
			continue;
		}

		entry_len = 0;
		do {
			entry_base += entry_len;
			memcpy(&entry_len, &dir_buf[entry_base], 2);
			if (entry_base + entry_len >= 4092) break;
		} while (entry_len > 0);

		if (entry_base + entry_len >= 4092) { //allocate new extents block and make this first entry
			uint32_t new_block_num = allocate_block(args);
			unsigned char new_block_buf[4096];
			readblock(fs->fd, new_block_buf, new_block_num);
			//initialize new final extents block
			memcpy(&new_block_buf[4092], &next_extents, 4);
			type_code = 3;
			memcpy(&new_block_buf[0], &type_code, 4);
			entry_len = 0;
			memcpy(&new_block_buf[4], &entry_len, 2);
			writeblock(fs->fd, new_block_buf, new_block_num);
			//append to dir extents list
			memcpy(&dir_buf[4092], &new_block_num, 4);
			writeblock(fs->fd, dir_buf, dir_num);
			dir_num = new_block_num;
			continue;
		}
		else { //entry_len == 0, found end of entries list
			int new_entry_len = len + 6;
			memcpy(&dir_buf[entry_base], &new_entry_len, 2);
			memcpy(&dir_buf[entry_base+2], &ino_map, 4);
			memcpy(&dir_buf[entry_base+6], name, (size_t)len);
			//metadata update
			uint16_t zero = 0;
			memcpy(&dir_buf[entry_base+new_entry_len], &zero, 2);
			uint64_t actual_size;
			memcpy(&actual_size, &ino_buf[48], 8);
			actual_size += new_entry_len;
			memcpy(&ino_buf[48], &actual_size, 8);
			update_tim(ino_buf, AMC_s);
			writeblock(fs->fd, ino_buf, parent_dir);
			writeblock(fs->fd, dir_buf, dir_num);
			break;
		}
	}
    return 0;
}

uint32_t remove_dir_entry(void *args, uint32_t parent_dir, const char *name) {
	printf("HELPER: remove directory entry\n");
    struct Args *fs = (struct Args*)args;
	uint32_t dir_num = parent_dir;
	unsigned char dir_buf[4096];
	uint32_t type_code;
	uint32_t block_num = 0;

	unsigned char ino_buf[4096];
	readblock(fs->fd, ino_buf, parent_dir);
	memcpy(&type_code, &ino_buf[0], 4);
	if (type_code != 2) return 0;

	uint32_t next_extents;
	int entry_base;
	uint16_t entry_len;
	uint32_t entry_inode;
	char *entry_name_buf = (char *) calloc(4086, sizeof(char));
	unsigned char *entry;
	while (dir_num) {
		readblock(fs->fd, dir_buf, dir_num);
		memcpy(&type_code, &dir_buf[0], 4);
		if (type_code == 2) {
			entry_base = 64;
		}
		else if (type_code == 3) {
			entry_base = 4;
		}
		else return 0;

		memcpy(&next_extents, &dir_buf[4092], 4);
		entry = &dir_buf[entry_base];
		
		memcpy(&entry_len, &entry[0], 2);
		while (entry_len > 0) {
			memcpy(&entry_inode, &entry[2], 4);
			memcpy(entry_name_buf, &entry[6], (size_t) entry_len);
			entry_name_buf[entry_len-6] = 0;
			if (strcmp(entry_name_buf, name) == 0) {
				block_num = entry_inode;
			}
			
			if (block_num) { //entry found, move remaining entries up
				int size_to_move = 4092-(entry_base+entry_len);
				memmove(&dir_buf[entry_base],&dir_buf[entry_base+entry_len],(size_t)size_to_move);
				//metadata to show that there are no more entries in the new space at the end
				uint16_t zero = 0;
				int new_end = entry_base + size_to_move;
				memcpy(&dir_buf[new_end], &zero, 2);
				uint64_t actual_size;
				memcpy(&actual_size, &ino_buf[48], 8);
				actual_size -= entry_len;
				memcpy(&ino_buf[48], &actual_size, 8);
				update_tim(ino_buf, AMC_s);
				writeblock(fs->fd, ino_buf, parent_dir);
			}

			entry_base += entry_len;
			entry = &dir_buf[entry_base];
			memcpy(&entry_len, &entry[0], 2);
			if (entry_base + entry_len >= 4092) break;
		}
		if (block_num) break;
		dir_num = next_extents;
	}
	free(entry_name_buf);
	writeblock(fs->fd, dir_buf, dir_num);
    return block_num;
}

/* actual read-write implementation */

int my_truncate(void *args, uint32_t block_num, off_t new_size) {
    struct Args *fs = (struct Args*)args;
    int fd = fs->fd;

    printf("TRUNCATE inode=%u new_size=%ld\n", block_num, (long)new_size);

    unsigned char block_buf[4096];
    readblock(fd, block_buf, block_num);

    uint32_t type_code;
    memcpy(&type_code, &block_buf[0], 4);
    if (type_code != 2){
        return -EINVAL;
    }
    uint16_t mode;
    memcpy(&mode, &block_buf[4], 2);
    if (!S_ISREG(mode)){
        perror("truncate: inode is not of type file");
        return -EINVAL;
    }

    uint64_t curr_size;
    memcpy(&curr_size, &block_buf[48], 8);

    uint64_t curr_blocks;
    memcpy(&curr_blocks, &block_buf[56], 8);

    uint32_t first_extents;
    memcpy(&first_extents, &block_buf[4092], 4);

    const uint64_t FIRST = 4028;
    const uint64_t EXT = 4084;

    if (new_size < 0){
        new_size = 0;
    }

    uint64_t new_sz = (uint64_t)new_size;

    uint64_t extents_needed;
    if (new_sz == 0 || new_sz <= FIRST){
        extents_needed = 0; // accounts for the inode
    }
    else{ // else need multiple blocks
        uint64_t extra = new_sz - FIRST;
        extents_needed = (extra + EXT - 1) / EXT;
    }

    uint64_t curr_extents = (curr_blocks > 0) ? curr_blocks - 1 : 0;

    // ALLOCATE extra extents (if needed)
    if (extents_needed > curr_extents){
        uint64_t to_add = extents_needed - curr_extents;

        uint32_t last_ext = 0;
        if (first_extents != 0){
            last_ext = first_extents;
            unsigned char extent_buff[4096];
            while (last_ext != 0){
                readblock(fd, extent_buff, last_ext);
                uint32_t next;
                memcpy(&next, &extent_buff[4092], 4);
                if (next == 0){
                    break;
                }
                last_ext = next;
            }
        }

        uint64_t i = 0;
        for (i = 0; i < to_add; i++){
            uint32_t new_block = allocate_block(args);
            if (new_block == 0){
                return -ENOSPC;
            }

            unsigned char new_buff[4096];
            memset(new_buff, 0, sizeof(new_buff));

            uint32_t type4 = 4;
            memcpy(new_buff, &type4, 4);

            uint32_t next = 0;
            memcpy(&new_buff[4092], &next, 4);
            writeblock(fd, new_buff, new_block);

            if (curr_extents == 0 && i == 0){
                // no extents yet
                first_extents = new_block;
                memcpy(&block_buf[4092], &first_extents, 4);
            }
            else{
                // link from previous extent
                unsigned char last_buff[4096];
                readblock(fd, last_buff, last_ext);
                memcpy(&last_buff[4092], &new_block, 4);
                writeblock(fd, last_buff, last_ext);
            }

            last_ext = new_block;
            curr_extents++;
        }
    }
    // FREE extra extents from the end
    else if (extents_needed < curr_extents){
        if (extents_needed == 0){
            // free extents
            uint32_t ext = first_extents;
            unsigned char extent_buff[4096];
            while (ext != 0){
                readblock(fd, extent_buff, ext);
                uint32_t next;
                memcpy(&next, &extent_buff[4092], 4);
                free_block(args, ext);
                ext = next;
            }

            uint32_t zero = 0;
            memcpy(&block_buf[4092], &zero, 4);

            first_extents = 0;
            curr_extents = 0;
        }
        else{
            // keep the needed extents and free the remaining ones
            uint32_t curr = first_extents;
            uint32_t last_keep = 0;
            uint32_t next = 0;
            unsigned char extent_buff[4096];

            uint64_t i = 0;
            for (i = 0; i < extents_needed; i++){
                last_keep = curr;
                readblock(fd, extent_buff, curr);
                memcpy(&next, &extent_buff[4092], 4);
                curr = next;
            }
            // free 'next' first
            uint32_t to_free = curr;

            readblock(fd, extent_buff, last_keep);

            uint32_t zero = 0;
            memcpy(&extent_buff[4092], &zero, 4);
            writeblock(fd, extent_buff, last_keep);

            // free the remaining extents
            unsigned char free_buff[4096];
            uint32_t fr = to_free;
            while(fr != 0){
                readblock(fd, free_buff, fr);
                uint32_t nxt;
                memcpy(&nxt, &free_buff[4092], 4);
                free_block(args, fr);
                fr = nxt;
            }
            curr_extents = extents_needed;
        }
    }

    // update the parent's actual size to the new_size
    memcpy(&block_buf[48], &new_sz, 8);

    // update the parent's number of allocated blocks to the new amount of blocks
    uint64_t num_blocks = curr_extents+1;
    memcpy(&block_buf[56], &num_blocks, 8);

    // ctime set to now, NOTE: it may need to be set to mtime given
    update_tim(block_buf, MC_s);

    writeblock(fd, block_buf, block_num);

    return 0;
}

int my_write(void *args, uint32_t block_num, const char *buff, size_t wr_len, off_t wr_offset){
    struct Args *fs = (struct Args*)args;
    int fd = fs->fd;

    printf("WRITE %d\n", block_num);

    if (wr_len == 0){
        return 0;
    }

    unsigned char block_buf[4096];
    readblock(fd, block_buf, block_num);

    uint32_t type_code;
    memcpy(&type_code, block_buf, 4);
    if (type_code != 2){
        return -EINVAL;
    }
    uint16_t mode;
    memcpy(&mode, &block_buf[4], 2);
    if (!S_ISREG(mode)){
        perror("write: inode mode is not of type file");
    }

    uint64_t curr_size;
    memcpy(&curr_size, &block_buf[48], 8);

    uint64_t write_start = (uint64_t)wr_offset;
    uint64_t write_end = write_start + wr_len;
    uint64_t full_write_size = write_end;

    // grow if needed
    if (full_write_size > curr_size){
        int res = my_truncate(args, block_num, (off_t)full_write_size);
        if (res < 0){
            return res;
        }
        // re-read inode to see updated size
        readblock(fd, block_buf, block_num);
    }

    const uint64_t FIRST = 4028;
    const uint64_t EXT = 4084;

    size_t bytes_remaining = wr_len;
    uint64_t file_pos = 0; // offset at each curreent block
    uint32_t curr_block = block_num;
    uint32_t next_extents = 0;

    unsigned char extent_buff[4096];
    unsigned char *block;
    int first_block = 1;

    while (bytes_remaining > 0 && curr_block != 0){
        int data_start;
        uint64_t cap;

        if (first_block) {
            block = block_buf;
            data_start = 64; // starts at byte 64 in inode
            cap = FIRST;
        
            memcpy(&next_extents, &block[4092], 4);
            first_block = 0;
        }
        else {
            readblock(fd, extent_buff, curr_block);
            block = extent_buff;

            data_start = 8; //starts at byte 8 in file extents
            cap = EXT;
            memcpy(&next_extents, &block[4092], 4);
        }

        uint64_t block_file_end = file_pos + cap;

        // if the whole block is before where we're supposed to write, skip over it
        if (block_file_end <= write_start) {
            file_pos = block_file_end;
            curr_block = next_extents;
            continue;
        }

        if (file_pos >= write_end) break;

        uint64_t wr_begin = (file_pos > write_start) ? file_pos : write_start;
        uint64_t wr_end = (block_file_end < write_end) ? block_file_end : write_end;
        if (wr_end > wr_begin){
            size_t write_len = (size_t)(wr_end - wr_begin);
            if (write_len > bytes_remaining){
                write_len = bytes_remaining;
            }

            size_t start_in_block = (size_t)(wr_begin - file_pos);
            size_t buff_offset = (size_t)(wr_begin - write_start);

            memcpy(block + data_start + start_in_block, buff + buff_offset, write_len);

            writeblock(fd, block, curr_block);

            bytes_remaining -= write_len;
        }

        file_pos = block_file_end;
        curr_block = next_extents;
    }

    uint64_t final_size;
    memcpy(&final_size, &block_buf[48], 8);

    if (full_write_size > final_size){
        final_size = full_write_size;
        memcpy(&block_buf[48], &final_size, 8);
    }

    // change mod and status times to current time
    update_tim(block_buf, MC_s);

    writeblock(fd, block_buf, block_num);
    
    // return bytes written
    return (int)(wr_len - bytes_remaining);
}

int my_chmod(void *args, uint32_t block_num, mode_t new_mode) {
	printf("MYCHMOD\n");
    struct Args *fs = (struct Args*)args;
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) { //only inodes
		return -EINVAL;
	}
	memcpy(&block_buf[4], &new_mode, 4);
	writeblock(fs->fd, block_buf, block_num);
    return 0;
}

int my_chown(void *args, uint32_t block_num, uid_t new_uid, gid_t new_gid) {
	printf("MYCHOWN\n");
    struct Args *fs = (struct Args*)args;
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) { //only inodes
		return -EINVAL;
	}
	memcpy(&block_buf[8], &new_uid, 4);
	memcpy(&block_buf[12], &new_gid, 4);
	writeblock(fs->fd, block_buf, block_num);
    return 0;
}

int my_utimens(void *args, uint32_t block_num, const struct timespec tv[2]) {
	printf("MYUTIMENS\n");
    struct Args *fs = (struct Args*)args;
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) { //only inodes
		return -EINVAL;
	}
	struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

	uint32_t atim_s = (tv) ? tv[0].tv_sec : now.tv_sec;
	uint32_t atim_ns = (tv) ? tv[0].tv_nsec : now.tv_nsec;
	uint32_t mtim_s = (tv) ? tv[1].tv_sec : now.tv_sec;
	uint32_t mtim_ns = (tv) ? tv[1].tv_nsec : now.tv_nsec;
	memcpy(&block_buf[24], &atim_s, 4);
	memcpy(&block_buf[28], &atim_ns, 4);
	memcpy(&block_buf[32], &mtim_s, 4);
	memcpy(&block_buf[36], &mtim_ns, 4);
	update_tim(block_buf, C_s);

	writeblock(fs->fd, block_buf, block_num);
    return 0;
}

int my_rmdir(void *args, uint32_t block_num, const char *name) {
	printf("MYRMDIR\n");
    struct Args *fs = (struct Args*)args;
	if (!name || !block_num) return -1;
	unsigned char parent_buf[4096];
	readblock(fs->fd, parent_buf, block_num);
	uint32_t type_code;
	uint16_t mode;
	memcpy(&type_code, &parent_buf[0], 4);
	memcpy(&mode, &parent_buf[4], 2);
	if (type_code != 2) return -EINVAL;
	else if ((mode & S_IFDIR) == 0) return -EINVAL;

	uint32_t rm_num = remove_dir_entry(args, block_num, name);
	if (!rm_num) {
		perror("rmdir: remove_dir_entry() failed");
		return -1;
	}
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, rm_num);
	memcpy(&type_code, &block_buf[0], 4);
	memcpy(&mode, &block_buf[4], 2);
	uint64_t actual_size;
	memcpy(&actual_size, &block_buf[48], 8);
	if (type_code != 2) return -EINVAL;
	else if ((mode & S_IFDIR) == 0) return -EINVAL;
	else if (actual_size > 0) return -1;

	free_block(args, rm_num);
    return 0;
}

int my_unlink(void *args, uint32_t block_num, const char *name) {
	printf("MYUNLINK\n");
    struct Args *fs = (struct Args*)args;
	if (!name || !block_num) return -1;
	unsigned char parent_buf[4096];
	readblock(fs->fd, parent_buf, block_num);
	uint32_t type_code;
	memcpy(&type_code, &parent_buf[0], 4);
	if (type_code != 2) return -EINVAL;

	uint32_t rm_num = remove_dir_entry(args, block_num, name);
	if (!rm_num) {
		perror("unlink: remove_dir_entry() failed");
		return -1;
	}

	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, rm_num);
	uint16_t nlink;
	memcpy(&nlink, &block_buf[6], 2);
	nlink--;
	memcpy(&block_buf[6], &nlink, 2);
	update_tim(block_buf, C_s);

	writeblock(fs->fd, block_buf, rm_num);

	if (nlink == 0) {
		free_block(args, rm_num);
	}
    return 0;
}

int my_mknod(void *args, uint32_t parent_block, const char *name, mode_t new_mode, dev_t new_dev) {
	printf("MYMKNOD\n");
    struct Args *fs = (struct Args*)args;
	//standard sanity check
	if (!name || !new_mode || !new_dev || !parent_block) return -1;
	unsigned char parent_buf[4096];
	readblock(fs->fd, parent_buf, parent_block);
	uint32_t type_code;
	memcpy(&type_code, &parent_buf[0], 4);
	if (type_code != 2) return -EINVAL;

	//actual code
	uint32_t bnum = allocate_block(args);
	unsigned char block_buf[4096];
	type_code = 2;
	fuse_context *context = fuse_get_context();
	uint32_t uid = context->uid;
	uint32_t gid = context->gid;
	uint32_t user_flags = 0;
	uint32_t next_extents = 0;
	uint16_t mode = new_mode;
	uint16_t nlink = 1;
    if ((mode & S_IFMT) == 0){
        mode |= S_IFREG;
    }
	memcpy(&block_buf[0], &type_code, 4);
	memcpy(&block_buf[4], &mode, 2);
	memcpy(&block_buf[6], &nlink, 2);
	memcpy(&block_buf[8], &uid, 4);
	memcpy(&block_buf[12], &gid, 4);
	memcpy(&block_buf[16], &new_dev, 4);
	memcpy(&block_buf[20], &user_flags, 4);
	memcpy(&block_buf[4092], &next_extents, 4);
	uint64_t size = 0;
	uint64_t blocks = 1;
	memcpy(&block_buf[48], &size, 8);
	memcpy(&block_buf[56], &blocks, 8);
	update_tim(block_buf, AMC_s);
	
	writeblock(fs->fd, block_buf, bnum);
	if (add_dir_entry(args, parent_block, strlen(name), bnum, name)) {
		perror("mknod: add_dir_entry() failed");
		return -1;
	}
    return 0;
}

int my_symlink(void *args, uint32_t parent_block, const char *name, const char *link_dest) {
	printf("MYSYMLINK\n");
    struct Args *fs = (struct Args*)args;
	//standard sanity check
	if (!name || !link_dest || !parent_block) return -1;
	unsigned char parent_buf[4096];
	readblock(fs->fd, parent_buf, parent_block);
	uint32_t type_code;
	memcpy(&type_code, &parent_buf[0], 4);
	if (type_code != 2) return -EINVAL;

	//actual code
	uint32_t bnum = allocate_block(args);
	unsigned char block_buf[4096];
	type_code = 2;
	fuse_context *context = fuse_get_context();
	uint32_t uid = context->uid;
	uint32_t gid = context->gid;
	uint32_t user_flags = 0;
	uint32_t rdev = 0;
	uint32_t next_extents = 0;
	uint16_t mode = S_IFLNK | 0744;
	uint16_t nlink = 1;
	memcpy(&block_buf[0], &type_code, 4);
	memcpy(&block_buf[4], &mode, 2);
	memcpy(&block_buf[6], &nlink, 2);
	memcpy(&block_buf[8], &uid, 4);
	memcpy(&block_buf[12], &gid, 4);
	memcpy(&block_buf[16], &rdev, 4);
	memcpy(&block_buf[20], &user_flags, 4);
	memcpy(&block_buf[4092], &next_extents, 4);
	uint64_t size = strlen(link_dest) + 1;
	uint64_t blocks = 1;
	memcpy(&block_buf[48], &size, 8);
	memcpy(&block_buf[56], &blocks, 8);
	update_tim(block_buf, AMC_s);
	
	writeblock(fs->fd, block_buf, bnum);
	if (add_dir_entry(args, parent_block, strlen(name), bnum, name)) {
		perror("symlink: add_dir_entry() failed");
		return -1;
	}

	my_write(args, bnum, link_dest, (size_t)(strlen(link_dest)), 0);
    return 0;
}

int my_mkdir(void *args, uint32_t parent_block, const char *name, mode_t new_mode) {
	printf("MYMKDIR\n");
    struct Args *fs = (struct Args*)args;
	if (!name || !new_mode || !parent_block) return -1;
	unsigned char parent_buf[4096];
	readblock(fs->fd, parent_buf, parent_block);
	uint32_t type_code;
	memcpy(&type_code, &parent_buf[0], 4);
	if (type_code != 2) return -EINVAL;

	//actual code
	uint32_t bnum = allocate_block(args);
	unsigned char block_buf[4096];
	type_code = 2;
	fuse_context *context = fuse_get_context();
	uint32_t uid = context->uid;
	uint32_t gid = context->gid;
	uint32_t user_flags = 0;
	uint32_t rdev = 0;
	uint32_t next_extents = 0;
	uint16_t mode = new_mode | S_IFDIR;
	uint16_t nlink = 1;
	memcpy(&block_buf[0], &type_code, 4);
	memcpy(&block_buf[4], &mode, 2);
	memcpy(&block_buf[6], &nlink, 2);
	memcpy(&block_buf[8], &uid, 4);
	memcpy(&block_buf[12], &gid, 4);
	memcpy(&block_buf[16], &rdev, 4);
	memcpy(&block_buf[20], &user_flags, 4);
	memcpy(&block_buf[4092], &next_extents, 4);
	uint64_t size = 0;
	uint64_t blocks = 1;
	memcpy(&block_buf[48], &size, 8);
	memcpy(&block_buf[56], &blocks, 8);
	update_tim(block_buf, AMC_s);
	
	writeblock(fs->fd, block_buf, bnum);
	if (add_dir_entry(args, parent_block, strlen(name), bnum, name)) {
		perror("mkdir: add_dir_entry() failed");
		return -1;
	}
    return 0;
}

int my_link(void *args, uint32_t parent_block, const char *name, uint32_t dest_block) {
	printf("MYLINK\n");
    struct Args *fs = (struct Args*)args;
	//sanity checks
	if (!parent_block || !name || !dest_block) return -1;
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, dest_block);
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) return -1;
	//create hard link
	uint16_t nlink;
	memcpy(&nlink, &block_buf[6], 2);
	if (add_dir_entry(args, parent_block, strlen(name), dest_block, name)) {
		perror("link: add_dir_entry() failed");
		return -1;
	}
	nlink++;
	memcpy(&block_buf[6], &nlink, 2);
	update_tim(block_buf, C_s);
	writeblock(fs->fd, block_buf, dest_block);
    return 0;
}

int my_rename(void *args, uint32_t old_parent, const char *old_name, uint32_t new_parent, const char *new_name) {
	printf("MYRENAME\n");
    //struct Args *fs = (struct Args*)args;
	if (!old_parent || !old_name || !new_parent || !new_name) return -1;

	uint32_t bnum = remove_dir_entry(args, old_parent, old_name);
	if (!bnum) {
		perror("rename: remove_dir_entry() failed");
		return -1;
	}
	if (add_dir_entry(args, new_parent, strlen(new_name), bnum, new_name)) {
		perror("rename: add_dir_entry() failed");
		return -1;
	}
    return 0;
}


// --------------------Read-only-------------------------------------

static void set_file_descriptor(void *args, int fd)
{
	struct Args *fs = (struct Args*)args;
	fs->fd = fd;
}

static int mygetattr(void *args, uint32_t block_num, struct stat *stbuf)
{
	if (!stbuf) return -1;
	struct Args *fs = (struct Args*)args;
	printf("MYGETATTR %d\n", block_num);

	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	
	uint32_t inode_num;
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code == 4) {
		inode_num = *((uint32_t *)&block_buf[4]);
		readblock(fs->fd, block_buf, inode_num);
	}
	else if (type_code == 2) {
		inode_num = block_num;
	}
	else {
		return -EINVAL;
	}

	uint16_t mode, nlink;
	uint32_t uid, gid, rdev, atime_s, atime_ns, mtime_s, mtime_ns, ctime_s, ctime_ns;
	uint64_t size, blocks;

	memcpy(&mode, &block_buf[4], 2);
	memcpy(&nlink, &block_buf[6], 2);
	memcpy(&uid, &block_buf[8], 4);
	memcpy(&gid, &block_buf[12], 4);
	memcpy(&rdev, &block_buf[16], 4);
	memcpy(&atime_s, &block_buf[24], 4);
	memcpy(&atime_ns, &block_buf[28], 4);
	memcpy(&mtime_s, &block_buf[32], 4);
	memcpy(&mtime_ns, &block_buf[36], 4);
	memcpy(&ctime_s, &block_buf[40], 4);
	memcpy(&ctime_ns, &block_buf[44], 4);
	memcpy(&size, &block_buf[48], 8);
	memcpy(&blocks, &block_buf[56], 8);

	timespec atim, mtim, ctim;
	atim.tv_sec = atime_s;
	atim.tv_nsec = atime_ns;
	mtim.tv_sec = mtime_s;
	mtim.tv_nsec = mtime_ns;
	ctim.tv_sec = ctime_s;
	ctim.tv_nsec = ctime_ns;

	stbuf->st_dev = 0; //idk yet
	stbuf->st_ino = (ino_t) inode_num;
	stbuf->st_mode = (mode_t) mode;
	stbuf->st_nlink = (nlink_t) nlink;
	stbuf->st_uid = (uid_t) uid;
	stbuf->st_gid = (gid_t) gid;
	stbuf->st_rdev = (dev_t) rdev;
	stbuf->st_size = (off_t) size;
	stbuf->st_atim = atim;
	stbuf->st_mtim = mtim;
	stbuf->st_ctim = ctim;
	stbuf->st_blksize = (blksize_t) 4096;
	stbuf->st_blocks = (blkcnt_t) blocks;

    return 0;
}

static int myreaddir(void *args, uint32_t block_num, void *buf, CPE453_readdir_callback_t cb)
{
	if (!buf) return -1;
	struct Args *fs = (struct Args*)args;
	printf("MYREADDIR %d\n", block_num);

	//assume given block_num will always be an inode
	int first_block = 1;
	unsigned char block_buf[4096];
	uint32_t bnum = block_num; //current bnum
	uint32_t type_code;

	uint32_t next_extents;
	uint16_t dir_entry_len;
	uint32_t dir_entry_inode;
	char *dir_entry_name_buf = (char *) calloc(4086, sizeof(char)); //hard-coded max length of name
	unsigned char *dir_entry;
	int entry_header_start;
	while (bnum) {
		readblock(fs->fd, block_buf, bnum);
		//printf("read block num %d\n", bnum);
		memcpy(&type_code, &block_buf[0], 4);
		
		if (type_code == 2) {
			if (first_block) first_block = 0;
			else return -EINVAL; //this should never happen
			
			uint16_t mode;
			memcpy(&mode, &block_buf[4], 2);
			if ((mode & S_IFDIR) == 0) {
				//throw error, inode is not for directory
				return -EINVAL;
			}
			entry_header_start = 64;
		}
		else if (type_code == 3) {
			if (first_block) return -EINVAL; //given block num is NOT inode block
			entry_header_start = 4;
		}
		else { //throw error, wrong block type
			return -EINVAL;
		}

		memcpy(&next_extents, &block_buf[4092], 4); //last four bytes of block
		dir_entry = &block_buf[entry_header_start];
		//int entrycount = 0;
		
		memcpy(&dir_entry_len, &dir_entry[0], 2);
		while (dir_entry_len > 0) {
			//printf("reading entry %d, len %d, starting point %d\n", ++entrycount, dir_entry_len, entry_header_start);
			memcpy(&dir_entry_inode, &dir_entry[2], 4);
			memcpy(dir_entry_name_buf, &dir_entry[6], (size_t) dir_entry_len);
			dir_entry_name_buf[dir_entry_len-6] = 0;

			cb(buf, dir_entry_name_buf, dir_entry_inode);
			
			entry_header_start += dir_entry_len;
			dir_entry = &block_buf[entry_header_start];
			if (entry_header_start < 4092) {//let's see if there are issues with this
				memcpy(&dir_entry_len, &dir_entry[0], 2);
			}
			else {
				//printf("finished reading at block num %d\n", bnum);
				break;
			}
		}
		bnum = next_extents;
	}
	free(dir_entry_name_buf);
    return 0;
}

static int myopen(void *args, uint32_t block_num)
{
	struct Args *fs = (struct Args*)args;
	printf("MYOPEN %d\n", block_num);
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	uint16_t mode;
	memcpy(&mode, &block_buf[4], 2);
	if (type_code != 2) {
		return -EINVAL;
	}
	else if (mode & S_IFDIR) {
		return -EINVAL;
	}
    return 0;
}

static int myread(void *args, uint32_t block_num, char *buf, size_t size, off_t offset)
{
	if ((!size) || (!buf)) return 0;
	
	struct Args *fs = (struct Args*)args;
	printf("MYREAD %d\n", block_num);

	int first_block = 1;
	unsigned char block_buf[4096];
	uint32_t bnum = block_num; //current bnum
	uint32_t type_code;
	signed int running_offset = offset;
	unsigned long size_read = 0;
	unsigned long remaining_size = size - size_read;

	while (bnum) {
		//printf("Remaining size: %d\n", (int)remaining_size);
		readblock(fs->fd, block_buf, bnum);
		memcpy(&type_code, &block_buf[0], 4);
		int contents_start;
		if (type_code == 2) {
			if (first_block) first_block = 0;
			else return -1; //this should never happen
			
			uint16_t mode;
			memcpy(&mode, &block_buf[4], 2);
			if ((mode & S_IFREG) == 0) {
				//throw error, inode is not regular file
				return -EINVAL;
			}
			contents_start = 64;
		}
		else if (type_code == 4) {
			if (first_block) {
				return -EINVAL; //given block num is NOT inode block
				// alternative: redirect back to file inode
				// memcpy(&bnum, &block_buf[4], 4);
				// continue;
			}
			contents_start = 8;
		}
		else { //throw error, wrong block type
			return -EINVAL;
		}

		uint32_t next_extents;
		memcpy(&next_extents, &block_buf[4092], 4); //last four bytes of block
		
		//if offset in file not located in current block, go to next
		if (contents_start + running_offset >= 4092) {
			if (next_extents == 0) {
				//printf("offset not in file\n");
				return 0;
			}
			//printf("offset not in current block %d, going to next block %d\n", bnum, next_extents);
			running_offset -= (4092 - contents_start);
			bnum = next_extents;
			continue;
		}

		//else, read
		if (contents_start + running_offset + remaining_size >= 4092) { //reading continues to other blocks
			memcpy(&buf[size_read], 
				&block_buf[contents_start+running_offset], 
				(size_t) 4092-(contents_start+running_offset));
			size_read += 4092-(contents_start+running_offset);
			//printf("read %d bytes from block %d, continue\n", 4092-(contents_start+running_offset), bnum);
			running_offset = 0; //read from start of next block
		}
		else { //last block to read
			memcpy(&buf[size_read], 
				&block_buf[contents_start+running_offset], 
				(size_t) remaining_size);
			size_read = size;
			//printf("read %d bytes from block %d, return\n", (int)remaining_size, bnum);
			break;
		}

		remaining_size = size - size_read;
		bnum = next_extents;
	}

    return size_read;
}

static int myreadlink(void *args, uint32_t block_num, char *buf, size_t size)
{
	struct Args *fs = (struct Args*)args;
	printf("MYREADLINK %d\n", block_num);

	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	uint16_t mode;
	memcpy(&mode, &block_buf[4], 2);
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) return -EINVAL; //verify that given block num is inode

	if ((mode & S_IFLNK) == 0) return -EINVAL; //given block is not symlink

	uint64_t path_size;
	memcpy(&path_size, &block_buf[48], 8);
	if (path_size > (uint64_t)size) path_size = (uint64_t)size - 1;

	memcpy(buf, &block_buf[64], (size_t)path_size);
	buf[path_size] = 0;

	return 0;
}


static uint32_t root_node(void *args)
{
	struct Args *fs = (struct Args*)args;
	uint32_t root_num;
	unsigned char super[4096];
	readblock(fs->fd, super, 0);
	memcpy(&root_num, &super[4088], 4);
	return root_num;
}

#ifdef  __cplusplus
extern "C" {
#endif

struct cpe453fs_ops *CPE453_get_operations(void)
{
	static struct cpe453fs_ops ops;
	static struct Args args;
	memset(&ops, 0, sizeof(ops));
	ops.arg = &args;

	ops.getattr = mygetattr;
	ops.readdir = myreaddir;
	ops.open = myopen;
	ops.read = myread;
	ops.readlink = myreadlink;
	ops.root_node = root_node;
	ops.set_file_descriptor = set_file_descriptor;

	ops.chmod = my_chmod;
	ops.chown = my_chown;
	ops.utimens = my_utimens;
	ops.rmdir = my_rmdir;
	ops.unlink = my_unlink;
	ops.mknod = my_mknod;
	ops.symlink = my_symlink;
	ops.mkdir = my_mkdir;
	ops.link = my_link;
	ops.rename = my_rename;
	ops.truncate = my_truncate;
	ops.write = my_write;

	return &ops;
}

#ifdef  __cplusplus
}
#endif

