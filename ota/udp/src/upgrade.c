#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h> 
#include <fcntl.h> 
#include <arpa/inet.h>
#include <sys/mman.h>
#include <time.h>
#include <netdb.h>


#define OP_FLASH_ERASE	        0xFE00EEEE // 开始升级
#define OP_WRITE_IAP		    0xFE00AAAA // 传输完成
#define OP_FRIMWARE_RESET		0xFE00BBBB // 重启

#define PACK_PREAMLE            0X484C
#define F_PACK                  0x0046
#define CMD_VERSION             0x0043

#define UDP_HEADER              8

// 升级文件
struct FirmwareFile
{
	uint32_t code;
    uint32_t len;
    uint32_t sent;
	uint32_t crc;
	uint8_t date[4];
	uint8_t unused[120];
	int8_t describe[512];
	uint8_t buffer[0];
};

// 升级数据包
struct FirmwarePart {
	uint32_t offset;
	uint32_t crc;
	uint32_t data[128];
};

// 响应包
struct FirmWriteResp {
	uint32_t offset;
    int32_t result;
	int8_t msg[128];
};


// 交互包头
struct UdpHeader
{
	uint16_t sign; //
	uint16_t cmd; //
	uint16_t sn; //
	uint16_t len; //
};


unsigned int stm32crc(const unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}

// 封包
int packUdp(int len, void* payload, void* buf)
{
	struct UdpHeader* hdr = (struct UdpHeader*)buf;
	hdr->sign = PACK_PREAMLE;
	hdr->cmd = F_PACK;
	hdr->sn = rand();
    hdr->len = len;

    int len4 = ((len+3)>>2)*4;
    memcpy(buf + UDP_HEADER, payload, len);

	unsigned int* pcrc = (unsigned int*)((uint8_t*)buf + UDP_HEADER + len4);
	*pcrc = stm32crc((unsigned int*)buf, len4 / 4 + 2);

    return len4 + 12;
}

// 解包
int unpackUdp(int len, uint8_t* buf, struct UdpHeader* hdr, uint8_t** payload)
{
    memcpy(hdr, buf, UDP_HEADER);
	if (hdr->sign != PACK_PREAMLE) 
    {
        return -1;
	}

	if (hdr->len + 12 != len)
	{
		return -2;
	}

	unsigned int* crc = (unsigned int*)(buf + UDP_HEADER + hdr->len);
	if (*crc != stm32crc((unsigned int*)buf, hdr->len / 4 + 2))
	{
		return -3;
	}

    *payload = buf + UDP_HEADER;

    return hdr->len;
}

// 命令交互
int udpTalk(int fdUdp, const char* devIp, int devPort, 
            int nSend, void* sendBuf, 
            int* nRecv, void* recvBuf)
{
    struct sockaddr_in sin1;
	socklen_t sin_size = sizeof(sin1);
	bzero( &sin1, sizeof(struct sockaddr_in) );
	sin1.sin_family = AF_INET;
	sin1.sin_addr.s_addr = inet_addr(devIp);
	sin1.sin_port   = htons(devPort); 

    uint8_t udpBuf[1024];
    int len = packUdp(nSend, sendBuf, udpBuf);
    struct UdpHeader* cmd = (struct UdpHeader*)udpBuf;

    // 发送
    sendto(fdUdp, (char*)udpBuf, len, 0, (struct sockaddr *)&sin1, sin_size); 

    int i;
    for (i=0; i<3; i++) 
    {
        struct timeval to;
        to.tv_sec = 1;
        to.tv_usec = 0;
            
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fdUdp, &readfds);

        int rt = select(fdUdp+1, &readfds, NULL, NULL, &to);
        if (rt <= 0) continue;

        if (FD_ISSET(fdUdp, &readfds) )
        {
            struct sockaddr_in remote; 
            socklen_t sin_size = sizeof(remote);

            uint8_t respBuf[1024];
            int len = recvfrom(fdUdp, (char*)respBuf, sizeof(respBuf), 0, 
                (struct sockaddr *)&remote, &sin_size); 

            struct UdpHeader resp;
            uint8_t* payload;
            rt = unpackUdp(len, respBuf, &resp, &payload);
            // printf("unpack len %d : %d payload %d\n", len, rt, resp.len);

            if (rt > 0 && resp.sn == cmd->sn)
            {
                *nRecv = resp.len;
                memcpy(recvBuf, payload, resp.len);
                return 0;
            }
            printf("unknown pack %x len %d\n", resp.cmd, resp.len);
        }
    }
    return -1;
}

int upgradeTalk(int fdUdp, const char* devIp, int devPort, 
    struct FirmwarePart* part,
    struct FirmWriteResp* resp)
{
    uint8_t respBuf[1024];
    int nResp;

    // 发送开始升级
    int rt = udpTalk(fdUdp, devIp, devPort, sizeof(struct FirmwarePart), part, &nResp, respBuf);
    if (rt != 0 || nResp < sizeof(struct FirmWriteResp)) 
    {
        printf("udp talk error %d %d\n", rt, nResp);
        return -1;
    }
    memcpy(resp, respBuf, sizeof(struct FirmWriteResp));

    // 检查结果
    if (resp->offset != part->offset)
    {
        printf("offset %x != %x\n", resp->offset, part->offset);
        return -2;
    }

    return 0;
}



// 从返回文本中提取关键字后的版本号（截取到 \r 或 \n 为止）
static int extract_version(const char* buf, const char* key, char* out, int out_sz)
{
    const char* p = strstr(buf, key);
    if (!p)
        return -1;
    p += strlen(key);
    while (*p == ' ' || *p == '\t')
        p++;
    int i = 0;
    while (p[i] != '\0' && p[i] != '\r' && p[i] != '\n' && i < out_sz - 1)
    {
        out[i] = p[i];
        i++;
    }
    out[i] = '\0';
    return 0;
}

// 查询版本号：发送 LVERSH 命令（cmd=0x0043），解析 MCU/MOTOR 版本
static void query_version(int fdUdp, const char* devIp, int devPort)
{
    uint8_t udpBuf[1024];
    struct UdpHeader* hdr = (struct UdpHeader*)udpBuf;
    hdr->sign = PACK_PREAMLE;
    hdr->cmd = CMD_VERSION;
    hdr->sn = rand();
    hdr->len = 8;

    uint8_t payload[8] = {0};
    memcpy(payload, "LVERSH", 6);
    memcpy(udpBuf + UDP_HEADER, payload, 8);

    unsigned int* pcrc = (unsigned int*)(udpBuf + UDP_HEADER + 8);
    *pcrc = stm32crc((unsigned int*)udpBuf, 8 / 4 + 2);

    struct sockaddr_in sin1;
    socklen_t sin_size = sizeof(sin1);
    bzero(&sin1, sizeof(sin1));
    sin1.sin_family = AF_INET;
    sin1.sin_addr.s_addr = inet_addr(devIp);
    sin1.sin_port = htons(devPort);

    sendto(fdUdp, (char*)udpBuf, UDP_HEADER + 8 + 4, 0, (struct sockaddr*)&sin1, sin_size);

    int i;
    for (i = 0; i < 3; i++)
    {
        struct timeval to;
        to.tv_sec = 1;
        to.tv_usec = 0;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fdUdp, &readfds);

        int rt = select(fdUdp + 1, &readfds, NULL, NULL, &to);
        if (rt <= 0) continue;

        if (FD_ISSET(fdUdp, &readfds))
        {
            struct sockaddr_in remote;
            socklen_t rsize = sizeof(remote);
            uint8_t respBuf[1024];
            int len = recvfrom(fdUdp, (char*)respBuf, sizeof(respBuf), 0, (struct sockaddr*)&remote, &rsize);

            struct UdpHeader resp;
            uint8_t* respayload;
            rt = unpackUdp(len, respBuf, &resp, &respayload);
            if (rt > 0 && resp.sn == hdr->sn)
            {
                char text[1024] = {0};
                int n = resp.len;
                if (n > (int)sizeof(text) - 1) n = sizeof(text) - 1;
                memcpy(text, respayload, n);

                char head[64] = {0};
                char board[64] = {0};

                if (extract_version(text, "MCU VERSION:", head, sizeof(head)) == 0)
                    printf("MCU VERSION: %s\n", head);
                else
                    printf("MCU VERSION: unknown\n");

                if (extract_version(text, "MOTOR VERSION:", board, sizeof(board)) == 0)
                    printf("MOTOR VERSION: %s\n", board);
                else
                    printf("MOTOR VERSION: unknown\n");

                return;
            }
        }
    }
    printf("query version timeout\n");
}

int Upgrade(const char* devIp, int devPort, int binLen, char* binBuf)
{
	int fdUdp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // 开始升级
    struct FirmwarePart start;
    start.offset = OP_FLASH_ERASE;	 // 擦除FLASH
    start.crc = 0xffffffff; //  
    start.data[0] = binLen; // 长度

    struct FirmWriteResp resp;
    printf("send erase length %d\n", binLen);
    int rt = upgradeTalk(fdUdp, devIp, devPort, &start, &resp); 
    if (rt != 0) {
        printf("send start, resp %d\n", rt);
        return -1;
    }
    printf("start return %d, resp %d : %s\n", rt, resp.result, resp.msg);

    // 循环发送数据文件
    uint32_t binOffset = 0;
    while (binOffset < binLen)
    {
        // 升级文件片段
        struct FirmwarePart part;
        part.offset = binOffset;	
        memcpy(part.data, binBuf+binOffset, 512); // 每次发送512字节
        part.crc = stm32crc(part.data, 128);

        while (1) 
        {
            // 发送文件片段
            printf("sending part offset %x\n", binOffset);
            rt = upgradeTalk(fdUdp, devIp, devPort, &part, &resp);
            if (rt != 0) {
                printf("send part offset %x return %d\n", binOffset, rt);
                continue;
            }
            printf("offset %x return %d : %s\n", binOffset, resp.result, resp.msg);
            if (resp.result == 0) break;
        }
        binOffset += 512;
    }

    // 传输完成
    struct FirmwarePart iap;
    iap.offset = OP_WRITE_IAP;// 传输完成
    iap.crc = 0xffffffff;
    iap.data[0] = binLen; // 文件长度

    printf("sending iap length %d\n", binLen);
    rt = upgradeTalk(fdUdp, devIp, devPort, &iap, &resp);
    if (rt != 0) {
        printf("send iap error %d\n", rt);
        return -1;
    }
    printf("iap return %d %s\n", resp.result, resp.msg);

    // 写 IAP 成功才重启
    if (resp.result != 0)
    {
        printf("iap write failed, skip reset\n");
        return -1;
    }

    // 重启前查询版本号
    query_version(fdUdp, devIp, devPort);
 
    struct FirmwarePart reset;
    reset.offset = OP_FRIMWARE_RESET; // 重启
    reset.crc = 0xffffffff;
    reset.data[0] = 0xabcd1234; //确认 

    printf("sending reset\n");
    upgradeTalk(fdUdp, devIp, devPort, &reset, &resp);

	return 0;
}



// 读取升级文件
int loadFirmware(const char* path, struct FirmwareFile** ppFF) 
{
    struct stat st;
    if ( stat(path, &st) != 0)
    {
        printf("can not read %s info\n", path);
        return -1;
    }

    FILE* fp = fopen(path, "rb");
    if (fp == NULL) {
        printf("can not open file %s\n", path);
        return -2;
    }

    struct FirmwareFile* binFile = (struct FirmwareFile*)malloc(st.st_size);
			
    fread(binFile, 1, st.st_size, fp);
    fclose(fp);
			
    if (binFile->code != 0xb18e03ea )
    {
        printf("file header code %x\n", binFile->code);
        return -3;
    }
    if ((binFile->len % 512) != 0 )
    {
        printf("len %d\n", binFile->len);
        return -4;
    }
    if (binFile->len + sizeof(struct FirmwareFile) != st.st_size) 
    {
        printf("len error %ld != %ld\n", binFile->len + sizeof(struct FirmwareFile), st.st_size);
        return -5;
    }
    if (binFile->crc != stm32crc((uint32_t*)(binFile->buffer), binFile->len/4)) 
    {
        printf("crc error\n");
        return -6;
    }
    *ppFF = binFile;
    return 0;
} 
	
int main(int argc, char* argv[])
{
    int devPort = -1;
    char devIp[128] = "";
    char binPath[125] = "";
    
    int ch; 
    while((ch = getopt(argc, argv, "d:p:f:")) != -1)
    { 
        switch(ch) {
            case 'd': strcpy(devIp, optarg); break;
            case 'p': devPort = atoi(optarg); break;
            case 'f': strcpy(binPath, optarg); break;
            default: break;
        }
    }

    if (devPort < 0 || strlen(devIp) == 0) 
    {
        printf("%s -d lidar_ip_address -p 6543 -f firmware.bin\n", argv[0]);
        return -1;
    }

    struct FirmwareFile* binFile;
    if ( loadFirmware(binPath, &binFile) != 0)
    {
        printf("load firmware %s fail\n", binPath);
        return -2;
    }
    printf("firmware date %d %d %d %d : %s\n", 
        binFile->date[0], binFile->date[1], binFile->date[2], binFile->date[3],
        binFile->describe);

	return Upgrade(devIp, devPort, binFile->len, binFile->buffer);
}

