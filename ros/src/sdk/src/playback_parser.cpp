
#include"../include/playback_parser.h"
#include<errno.h>
#include"../include/usererror.h" 

// ======================
// 初始化解析上下文
// ======================
int init_context(ParseContext* ctx, const char* filename) {
    // 打开文件
    ctx->fd = open(filename, O_RDONLY);
    if (ctx->fd == -1) {
        perror("open failed");
        return -1;
    }

    // 获取文件大小
    struct stat st;
    if (fstat(ctx->fd, &st) == -1) {
        perror("fstat failed");
        close(ctx->fd);
        return -1;
    }

    ctx->file_size = st.st_size;

    ctx->processed_size = 0;
    ctx->current_offset = 0;
    ctx->mmap_ptr = NULL;
    ctx->mmap_size = 0;
    
    // 初始化统计
    ctx->bytes_processed = 0;
    
    return 0;
}

// ======================
// 清理资源
// ======================
void destroy_context(ParseContext* ctx) {
    if (ctx->mmap_ptr) {
        munmap(ctx->mmap_ptr, ctx->mmap_size);
    }
    if (ctx->fd != -1) {
        close(ctx->fd);
    }
}
// 获取系统页大小
long get_page_size() {
     long page_size = 0;
    if (page_size == 0) {
        page_size = sysconf(_SC_PAGESIZE);
        if (page_size == -1) {
            perror("sysconf failed");
            return -1;
        }
    }
    return page_size;
}
// ======================
// 映射下一个文件块
// ======================
int map_next_chunk(ParseContext* ctx) {
    long page_size = get_page_size();
    if (page_size <= 0) {
        return -1;
    }
    
    // 计算本次映射大小
    size_t remaining = ctx->file_size - ctx->processed_size;
    if (remaining == 0) {
        return 0; // 文件已处理完
    }
    
    // 确保偏移量对齐
    size_t aligned_offset = (ctx->processed_size / page_size) * page_size;
    size_t offset_adjustment = ctx->processed_size - aligned_offset;
    
    // 计算映射大小（包含对齐调整）
    size_t map_size = remaining + offset_adjustment;
    if (map_size > MEMORY_MAP_SIZE) {
        map_size = MEMORY_MAP_SIZE;
    }
    
    // 确保映射大小是页大小的倍数
    map_size = ((map_size + page_size - 1) / page_size) * page_size;
    if (map_size == 0) {
        // 当剩余数据小于页大小时，映射最小大小
        map_size = page_size;
    }
    
    // 检查映射是否超出文件范围
    if (aligned_offset + map_size > ctx->file_size) {
        map_size = ctx->file_size - aligned_offset;
    }
    
    // 取消之前的映射
    if (ctx->mmap_ptr) {
        munmap(ctx->mmap_ptr, ctx->mmap_size);
    }
    
    // 创建新映射
    ctx->mmap_ptr = mmap(NULL, map_size, PROT_READ, MAP_PRIVATE, ctx->fd, aligned_offset);
    if (ctx->mmap_ptr == MAP_FAILED) {
        // 处理小文件映射的特殊情况
        if (map_size < (size_t)page_size) {
            // 尝试直接映射剩余部分
            map_size = remaining;
            ctx->mmap_ptr = mmap(NULL, map_size, PROT_READ, MAP_PRIVATE, ctx->fd, aligned_offset);
            if (ctx->mmap_ptr == MAP_FAILED) {
                fprintf(stderr, "mmap failed for small chunk: %s (offset: %zu, size: %zu)\n", 
                        strerror(errno), aligned_offset, map_size);
                return -1;
            }
        } else {
            fprintf(stderr, "mmap failed: %s (offset: %zu, size: %zu)\n", 
                    strerror(errno), aligned_offset, map_size);
            return -1;
        }
    }
    
    ctx->mmap_size = map_size;
    ctx->last_mapped_offset = aligned_offset;
    ctx->current_offset = offset_adjustment;
    
    return 0;
}

//hex2byte
static const uint8_t hex2val[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   255, 255, 255, 255, 255, 255,
    255, 10,  11,  12,  13,  14,  15,  255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 10,  11,  12,  13,  14,  15,  255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

uint16_t hex4_to_u16le(const char *p)
{
    const uint8_t *q = (const uint8_t *)p;
    uint8_t lo = hex2val[q[0]];
    uint8_t hi = hex2val[q[1]];
    uint8_t msb = hex2val[q[2]];
    uint8_t lsb = hex2val[q[3]];
    return (uint16_t)((msb << 12) | (lsb << 8) | (lo << 4) | hi);
}

uint16_t hex_str_to_bytes(const char *hex, size_t len, uint8_t *out)
{
    if (!hex || !out || (len & 1)) return -1;

    const uint8_t *h = (const uint8_t *)hex;

    for (size_t i = 0; i < len; i += 2) {
        uint8_t hi = hex2val[h[i]];
        uint8_t lo = hex2val[h[i + 1]];
        if (hi > 0xF0 || lo > 0xF0) return -1;
        out[i / 2] = (hi << 4) | lo;
    }
    return len / 2;
}