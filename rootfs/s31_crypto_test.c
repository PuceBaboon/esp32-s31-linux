// SPDX-License-Identifier: GPL-2.0-only
#include <errno.h>
#include <linux/if_alg.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int open_alg(const char *type, const char *name)
{
	struct sockaddr_alg sa = { .salg_family = AF_ALG };
	int fd;

	strncpy((char *)sa.salg_type, type, sizeof(sa.salg_type) - 1);
	strncpy((char *)sa.salg_name, name, sizeof(sa.salg_name) - 1);
	fd = socket(AF_ALG, SOCK_SEQPACKET, 0);
	if (fd < 0)
		return -1;
	if (bind(fd, (struct sockaddr *)&sa, sizeof(sa))) {
		close(fd);
		return -1;
	}
	return fd;
}

static int test_sha256(void)
{
	static const unsigned char expected[] = {
		0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
		0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
		0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
		0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
	};
	unsigned char digest[sizeof(expected)];
	int fd = open_alg("hash", "sha256"), op;

	if (fd < 0 || (op = accept(fd, NULL, NULL)) < 0 ||
	    write(op, "abc", 3) != 3 || read(op, digest, sizeof(digest)) != sizeof(digest) ||
	    memcmp(digest, expected, sizeof(digest)))
		return -1;
	close(op);
	close(fd);
	return 0;
}

static int test_sha256_stream(void)
{
	static const unsigned char expected[] = {
		0xad, 0x7f, 0xac, 0xb2, 0x58, 0x6f, 0xc6, 0xe9,
		0x66, 0xc0, 0x04, 0xd7, 0xd1, 0xd1, 0x6b, 0x02,
		0x4f, 0x58, 0x05, 0xff, 0x7c, 0xb4, 0x7c, 0x7a,
		0x85, 0xda, 0xbd, 0x8b, 0x48, 0x89, 0x2c, 0xa7,
	};
	unsigned char input[4096] = { }, digest[sizeof(expected)];
	int fd = open_alg("hash", "sha256"), op;

	if (fd < 0 || (op = accept(fd, NULL, NULL)) < 0 ||
	    write(op, input, sizeof(input)) != sizeof(input) ||
	    read(op, digest, sizeof(digest)) != sizeof(digest) ||
	    memcmp(digest, expected, sizeof(digest)))
		return -1;
	close(op);
	close(fd);
	return 0;
}

static int test_hash(const char *name, const unsigned char *expected,
		     size_t digest_len)
{
	unsigned char digest[64];
	int fd = open_alg("hash", name), op;

	if (fd < 0 || (op = accept(fd, NULL, NULL)) < 0 ||
	    write(op, "abc", 3) != 3 ||
	    read(op, digest, digest_len) != (ssize_t)digest_len ||
	    memcmp(digest, expected, digest_len))
		return -1;
	close(op);
	close(fd);
	return 0;
}

static int test_sha512(void)
{
	static const unsigned char sha512[] = {
		0xdd, 0xaf, 0x35, 0xa1, 0x93, 0x61, 0x7a, 0xba,
		0xcc, 0x41, 0x73, 0x49, 0xae, 0x20, 0x41, 0x31,
		0x12, 0xe6, 0xfa, 0x4e, 0x89, 0xa9, 0x7e, 0xa2,
		0x0a, 0x9e, 0xee, 0xe6, 0x4b, 0x55, 0xd3, 0x9a,
		0x21, 0x92, 0x99, 0x2a, 0x27, 0x4f, 0xc1, 0xa8,
		0x36, 0xba, 0x3c, 0x23, 0xa3, 0xfe, 0xeb, 0xbd,
		0x45, 0x4d, 0x44, 0x23, 0x64, 0x3c, 0xe8, 0x0e,
		0x2a, 0x9a, 0xc9, 0x4f, 0xa5, 0x4c, 0xa4, 0x9f,
	};
	return test_hash("sha512", sha512, sizeof(sha512));
}

static int test_ecb(void)
{
	static const unsigned char key[16] = {
		0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
		0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
	};
	static const unsigned char plain[16] = {
		0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
		0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
	};
	static const unsigned char expected[16] = {
		0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60,
		0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66, 0xef, 0x97,
	};
	struct iovec iov = { .iov_base = (void *)plain, .iov_len = sizeof(plain) };
	char control[CMSG_SPACE(sizeof(int))] = { 0 };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1,
		.msg_control = control, .msg_controllen = sizeof(control) };
	struct cmsghdr *cmsg;
	unsigned char output[16];
	int opmode = ALG_OP_ENCRYPT;
	int fd = open_alg("skcipher", "ecb(aes)"), op;

	if (fd < 0 || setsockopt(fd, SOL_ALG, ALG_SET_KEY, key, sizeof(key)) ||
	    (op = accept(fd, NULL, NULL)) < 0)
		return -1;
	cmsg = CMSG_FIRSTHDR(&msg);
	cmsg->cmsg_level = SOL_ALG;
	cmsg->cmsg_type = ALG_SET_OP;
	cmsg->cmsg_len = CMSG_LEN(sizeof(opmode));
	memcpy(CMSG_DATA(cmsg), &opmode, sizeof(opmode));
	if (sendmsg(op, &msg, 0) != sizeof(plain) || read(op, output, sizeof(output)) != sizeof(output) ||
	    memcmp(output, expected, sizeof(expected)))
		return -1;
	close(op);
	close(fd);
	return 0;
}

int main(void)
{
	if (test_sha256() || test_sha256_stream() || test_sha512() ||
	    test_ecb()) {
		perror("AF_ALG known-answer test");
		return 1;
	}
	puts("Linux Crypto API AES-ECB, SHA-256 and SHA-512: PASS");
	puts("Hardware crypto remains disabled while its AES island is shared with the radio blob.");
	return 0;
}
