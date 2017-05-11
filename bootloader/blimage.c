/*
  create a image file for OTA update from txmain.bin
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <crc.h>
#include <arpa/inet.h>

int main(void)
{
    int fd_in = open("txmain.bin", O_RDONLY);
    int fd_out = open("txmain.img", O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (fd_in == -1) {
        printf("failed to open txmain.bin\n");
        exit(1);
    }
    if (fd_out == -1) {
        printf("failed to open txmain.img\n");
        exit(1);
    }

    struct stat st;
    fstat(fd_in, &st);
    uint16_t size = st.st_size;

    uint8_t *b = malloc(size);
    if (read(fd_in, b, st.st_size) != size) {
        printf("Failed to read %u bytes\n", (unsigned)size);
    }

    uint32_t crc = crc_crc32(b, size);
    printf("size: %u crc:0x%08x\n", size, crc);

    uint32_t crc_swapped = htonl(crc);
    uint16_t size_swapped = htons(size);

    if (write(fd_out, &size_swapped, sizeof(size_swapped)) != sizeof(size_swapped) ||
        write(fd_out, &crc_swapped, sizeof(crc_swapped)) != sizeof(crc_swapped) ||
        write(fd_out, b, size) != size) {
        printf("write failed\n");
        exit(1);
    }
    close(fd_in);
    close(fd_out);
    return 0;
}
