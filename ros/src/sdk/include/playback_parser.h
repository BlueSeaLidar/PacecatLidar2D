#ifndef __PLAYBACK_PARSER_H__
#define __PLAYBACK_PARSER_H__
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>

// ======================
// PCAP文件格式定义 PCAP 文件头
// ======================
typedef struct pcap_file_header {
    uint32_t magic_number;    // 0xa1b2c3d4
    uint16_t version_major;   // 2
    uint16_t version_minor;   // 4
    int32_t  thiszone;        // GMT to local correction
    uint32_t sigfigs;         // accuracy of timestamps
    uint32_t snaplen;         // max length of captured packets
    uint32_t linktype;        // data link type (1 = Ethernet)
} pcap_file_header_t;

// PCAP 包头
typedef struct pcap_packet_header {
    uint32_t ts_sec;         // timestamp seconds
    uint32_t ts_usec;        // timestamp microseconds
    uint32_t incl_len;       // number of octets of packet saved in file
    uint32_t orig_len;       // actual length of packet
} pcap_packet_header_t;

// ======================
// 配置参数
// ======================
#define MEMORY_MAP_SIZE   (1024 * 1024 * 1024) // 256MB内存映射块

// ======================
// 解析上下文
// ======================
typedef struct {
    int fd;                 // 文件描述符
    size_t file_size;       // 文件总大小
    size_t processed_size;  // 已处理大小
    size_t current_offset;  // 当前偏移
    void* mmap_ptr;         // 内存映射指针
    size_t mmap_size;       // 当前映射大小
    size_t last_mapped_offset; // 上次映射的起始偏移
    
    // 统计信息
    uint64_t bytes_processed;
    
    // 实时打印控制
    volatile sig_atomic_t running;
} ParseContext;

// ======================
// 函数声明
// ======================
int init_context(ParseContext* ctx, const char* filename);
void destroy_context(ParseContext* ctx);
int map_next_chunk(ParseContext* ctx);
long get_page_size();

//hex2byte
uint16_t hex4_to_u16le(const char *p);
uint16_t hex_str_to_bytes(const char *hex, size_t len, uint8_t *out);

#endif