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

static void set_file_descriptor(void *args, int fd)
{
	struct Args *fs = (struct Args*)args;
	fs->fd = fd;
}

static int mygetattr(void *args, uint32_t block_num, struct stat *stbuf)
{
	struct Args *fs = (struct Args*)args;
	printf("GETATTR %d\n", block_num);

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
		return -1;
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
	struct Args *fs = (struct Args*)args;

	//assume given block_num will always be an inode
	int first_block = 1;
	unsigned char block_buf[4096];
	uint32_t bnum = block_num; //current bnum
	uint32_t type_code;

	while (bnum) {
		readblock(fs->fd, block_buf, bnum);
		memcpy(&type_code, &block_buf[0], 4);
		int entry_header_start;
		if (type_code == 2) {
			if (first_block) first_block = 0;
			else return -1; //this should never happen
			
			uint16_t mode;
			memcpy(&mode, &block_buf[4], 2);
			if ((mode & S_IFDIR) == 0) {
				//throw error, inode is not for directory
				return -1;
			}
			entry_header_start = 64;
		}
		else if (type_code == 3) {
			if (first_block) return -1; //given block num is NOT inode block
			entry_header_start = 4;
		}
		else { //throw error, wrong block type
			return -1;
		}

		uint32_t next_extents;
		memcpy(&next_extents, &block_buf[4092], 4); //last four bytes of block
		uint16_t dir_entry_len;
		uint32_t dir_entry_inode;
		char *dir_entry_name_buf = (char *) calloc(4086, sizeof(char)); //hard-coded max length of name
		unsigned char *dir_entry = &block_buf[entry_header_start];
		
		memcpy(&dir_entry_len, &dir_entry[0], 2);
		while (dir_entry_len > 0) {
			memcpy(&dir_entry_inode, &dir_entry[2], 4);
			memcpy(dir_entry_name_buf, &dir_entry[6], (size_t) dir_entry_len);
			dir_entry_name_buf[dir_entry_len] = 0;

			cb(buf, dir_entry_name_buf, dir_entry_inode);
			
			entry_header_start += dir_entry_len + 6;
			dir_entry = &block_buf[entry_header_start];
			if (entry_header_start < 4092) //let's see if there are issues with this
				memcpy(&dir_entry_len, &dir_entry[0], 2);
			else
				free(dir_entry_name_buf);
				break;
		}

		bnum = next_extents;
	}

    return 0;
}

static int myopen(void *args, uint32_t block_num)
{
	struct Args *fs = (struct Args*)args;
	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) { //anything else to check?
		//throw error
		return -1;
	}

    return 0;
}

static int myread(void *args, uint32_t block_num, char *buf, size_t size, off_t offset)
{
	struct Args *fs = (struct Args*)args;
	int res = -1;

	int first_block = 1;
	unsigned char block_buf[4096];
	uint32_t bnum = block_num; //current bnum
	uint32_t type_code;
	signed int running_offset = offset;
	unsigned long size_read = 0;
	unsigned long remaining_size = size - size_read;

	while (bnum) {
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
				return -1;
			}
			contents_start = 64;
		}
		else if (type_code == 4) {
			if (first_block) {
				return -1; //given block num is NOT inode block
				// alternative: redirect back to file inode
				// memcpy(&bnum, &block_buf[4], 4);
				// continue;
			}
			contents_start = 8;
		}
		else { //throw error, wrong block type
			return -1;
		}

		uint32_t next_extents;
		memcpy(&next_extents, &block_buf[4092], 4); //last four bytes of block
		
		//if offset in file not located in current block, go to next
		if (contents_start + running_offset >= 4092) {
			running_offset -= (4096 - contents_start);
			bnum = next_extents;
			continue;
		}

		//else, read
		if (contents_start + running_offset + remaining_size >= 4092) { //reading continues to other blocks
			memcpy(&buf[size_read], 
				&block_buf[contents_start+running_offset], 
				(size_t) 4092-(contents_start+running_offset));
			size_read += 4092-(contents_start+running_offset);
			running_offset = 0; //read from start of next block
		}
		else { //last block to read
			memcpy(&buf[size_read], 
				&block_buf[contents_start+running_offset], 
				(size_t) remaining_size);
			size_read = size;
			res = 0;
			break;
		}

		remaining_size = size - size_read;
		bnum = next_extents;
	}

    return res;
}

static int myreadlink(void *args, uint32_t block_num, char *buf, size_t size)
{
	struct Args *fs = (struct Args*)args;

	unsigned char block_buf[4096];
	readblock(fs->fd, block_buf, block_num);
	uint32_t type_code;
	uint16_t mode;
	memcpy(&mode, &block_buf[4], 2);
	memcpy(&type_code, &block_buf[0], 4);
	if (type_code != 2) return -1; //verify that given block num is inode

	if ((mode & S_IFLNK) == 0) return -1; //given block is not symlink

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
	uint32_t *root_num;
	unsigned char super[10240];
	readblock(fs->fd, super, 0);
	root_num = (uint32_t *)(super + 4088);
	return *root_num;
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

	return &ops;
}

#ifdef  __cplusplus
}
#endif

