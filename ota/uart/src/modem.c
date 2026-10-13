/*
 * Minimalistic implementation of the XModem/YModem protocol suite, including
 * a compact version of an CRC16 algorithm. The code is just enough to upload
 * an image to an MCU that bootstraps itself over an UART.
 *
 * Copyright (c) 2014 Daniel Mack <github@zonque.org>
 *
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#define X_STX 0x01
#define X_EOF 0x04
#define X_ACK 0x06
#define X_NAK 0x15

#define min(a, b)       ((a) < (b) ? (a) : (b))

#pragma pack (push,1)  
struct xmodem_chunk {
    uint8_t start;
    uint8_t block;
    uint8_t block_neg;
    uint8_t payload[128];
    uint16_t crc;
};// __attribute__((packed));
#pragma pack (pop)  


#define CRC_POLY 0x1021
#define CRC16POLY 0x1021

static uint16_t crc_update(uint16_t crc_in, int incr)
{
    uint16_t xor_tmp = crc_in >> 15;
    uint16_t out = crc_in << 1;

    if (incr)
        out++;

    if (xor_tmp)
        out ^= CRC_POLY;

    return out;
}

static uint16_t crc16(const uint8_t* data, uint16_t size)
{
    uint16_t crc, i;

    for (crc = 0; size > 0; size--, data++)
        for (i = 0x80; i; i >>= 1)
            crc = crc_update(crc, *data & i);

    for (i = 0; i < 16; i++)
        crc = crc_update(crc, 0);

    return crc;
}
static uint16_t swap16(uint16_t in)
{
    return (in >> 8) | ((in & 0xff) << 8);
}

uint16_t calcrc(uint8_t* ptr, int count)
{
    uint16_t crc, cmpt;

    crc = 0;
    //* For  all char
    while (--count >= 0)
    {

        crc = crc ^ (int)*ptr++ << 8;
        //* For All bit
        for (cmpt = 0; cmpt < 8; cmpt++)
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ CRC16POLY;
            else
                crc = crc << 1;
        }//* end bit
    }//* Frame end

    return (crc & 0xFFFF);
}

enum {
    PROTOCOL_XMODEM,
    PROTOCOL_YMODEM,
};

int read_char(int fd, uint8_t*, int timeout);
int xread(int fd, void*, size_t sz);
int xwrite(int fd, void* buf, size_t sz);
static int xymodem_send(int serial_fd, size_t len, const uint8_t* buf, int protocol, int wait)
{
    int ret;
    uint8_t answer;
    uint8_t eof = X_EOF;
    struct xmodem_chunk chunk;
    int skip_payload = 0;

    if (wait) {
        printf("wait for lidar transfer signal ...\n");
        // fflush(stdout);

        do {
            //ret = read_char(serial_fd, &answer, 10000);
            ret = xread(serial_fd, &answer, 1);
            if (ret != sizeof(answer)) {
                //printf("read error\n");
                continue;
                //return -1;// errno;
            }
           // printf("%c", answer);
           // fflush(stdout);
        } while (answer != 'C');
        // printf("done.\n");
    }

    printf("Sending \n");

    if (protocol == PROTOCOL_YMODEM) {
        //strncpy((char*)chunk.payload, "filename", sizeof(chunk.payload));
        chunk.block = 0;
        skip_payload = 1;
    }
    else {
        chunk.block = 1;
    }

    chunk.start = X_STX;

    while (len) {
        size_t z = 0;
        int next = 0;
        char status;

        if (!skip_payload) {
            z = min(len, sizeof(chunk.payload));
            memcpy(chunk.payload, buf, z);
            memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);
        }
        else {
            skip_payload = 0;
        }

        //chunk.crc = swap16(crc16(chunk.payload, sizeof(chunk.payload)));
        chunk.crc = swap16(calcrc(chunk.payload, sizeof(chunk.payload)));
        chunk.block_neg = 0xff - chunk.block;

        ret = xwrite(serial_fd, &chunk, sizeof(chunk));
        if (ret != sizeof(chunk))
            return -2;// errno;

        usleep(10000);
        //ret = read_char(serial_fd, &answer, 3000);
        ret = xread(serial_fd, &answer, 1);
        if (ret != sizeof(answer))
            return -3;// errno;
        switch (answer) {
        case X_NAK:
            status = 'N';
            break;
        case X_ACK:
            status = '.';
            next = 1;
            break;
        default:
            status = '?';
            break;
        }

        printf("%c", status);
        fflush(stdout);

        if (next) {
            chunk.block++;
            len -= z;
            buf += z;
        }
    }

    ret = xwrite(serial_fd, &eof, sizeof(eof));
    if (ret != sizeof(eof))
        return -4;// errno;

    /* send EOT again for YMODEM */
    if (protocol == PROTOCOL_YMODEM) {
        ret = xwrite(serial_fd, &eof, sizeof(eof));
        if (ret != sizeof(eof))
            return -5;// errno;
    }

    printf("transfer bin finished\n");

    return 0;
}

int xmodem_send(int serial_fd, size_t len, const uint8_t* buf )
{
    return xymodem_send(serial_fd, len, buf, PROTOCOL_XMODEM, 1);
}

#if 0
static int open_serial(const char* path, int baud)
{
    int fd;
    struct termios tty;

    fd = open(path, O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -errno;
    }

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -errno;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo,
                                                    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN] = 1;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
                                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -errno;
    }

    return fd;
}

static void dump_serial(int serial_fd)
{
    char in;

    for (;;) {
        read(serial_fd, &in, sizeof(in));
        printf("%c", in);
        fflush(stdout);
    }
}

int main(int argc, char** argv)
{
    int a, ret, serial_fd;

    serial_fd = open_serial("/dev/ttyUSB0", 115200);
    if (serial_fd < 0)
        return -errno;

    ret = xymodem_send(serial_fd, argv[1], PROTOCOL_XMODEM, 1);
    if (ret < 0)
        return ret;

    ret = xymodem_send(serial_fd, argv[2], PROTOCOL_YMODEM, 1);
    if (ret < 0)
        return ret;

    sleep(4);

    ret = xymodem_send(serial_fd, argv[1], PROTOCOL_YMODEM, 0);
    if (ret < 0)
        return ret;

    sleep(3);

    ret = xymodem_send(serial_fd, argv[2], PROTOCOL_YMODEM, 0);
    if (ret < 0)
        return ret;

    dump_serial(serial_fd);

    return 0;
}
#endif