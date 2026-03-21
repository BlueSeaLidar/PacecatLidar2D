#include "../include/reader.h"
#include "../include/playback_parser.h"
#include <cstdio>
#include <cstdint>

// 进度打印
#define PRINT_PROGRESS(LAST_PCT, PROCESSED, TOTAL, THRESHOLD, LABEL)          \
do {                                                                          \
    double pct_ = 100.0 * (PROCESSED) / (TOTAL);                              \
    if (pct_ - (LAST_PCT) >= (THRESHOLD) || pct_ >= 99.999) {                 \
        double mb_done_ = (PROCESSED) / 1048576.0;                            \
        double mb_tot_  = (TOTAL)   / 1048576.0;                              \
        DEBUG("%s progress: %.1f %%  (%.2f / %.2f MB)",                       \
              (LABEL), pct_, mb_done_, mb_tot_);                              \
        (LAST_PCT) = pct_;                                                    \
    }                                                                         \
} while(0)

struct PlaybackInfo 
{
	Parser* hParser; 
	PubHub* hPublish;
    std::string log_path;
	std::string log_file_format;
    ParseContext ctx = {};
    pthread_t thrPlayback;
};

void *PlaybackPcapThreadProc(void *p);
void *PlaybackRawThreadProc(void *p);
void *PlaybackHexThreadProc(void *p);

HReader StartPlaybackReader(Parser *hParser, PubHub* hPub, std::string logpath, std::string logfileformat)
{
	PlaybackInfo *info = new PlaybackInfo;
    info->hParser = hParser;
    info->hPublish = hPub;
    info->log_path = logpath;
	info->log_file_format = logfileformat;

	if(logfileformat=="pcap")
	{
		pthread_create(&info->thrPlayback, NULL, PlaybackPcapThreadProc, info);	
	}
	else if(logfileformat=="bin")
	{
		pthread_create(&info->thrPlayback, NULL, PlaybackRawThreadProc, info);	
	}
	else if(logfileformat=="hex")
	{
		pthread_create(&info->thrPlayback, NULL, PlaybackHexThreadProc, info);	
	}
	else
	{
		DEBUG("unknown file format:%s", logfileformat.c_str());
	}
	
	return info;
}

void StopPlaybackReader(HReader hr)
{
	PlaybackInfo *info = (PlaybackInfo *)hr;
    info->ctx.running = 0;
    pthread_join(info->thrPlayback, NULL);
	delete info;
}

void *PlaybackPcapThreadProc(void *p)
{
    PlaybackInfo *info = (PlaybackInfo *)p;
	ParseContext& ctx = info->ctx;

	DEBUG("pcap logpath:%s", info->log_path.c_str());

	if (init_context(&ctx, info->log_path.c_str()) != 0)
	{
        DEBUG("init_context error");
		return NULL;
	}

	DEBUG("pcap log file size:%.2fMB", (double)ctx.file_size / (1024 * 1024));

	ctx.running = 1;
	while (ctx.running && ctx.processed_size < ctx.file_size)
	{
		if (map_next_chunk(&ctx) != 0)
		{
			break;
		}

		const uint8_t *data = (const uint8_t *)ctx.mmap_ptr;
		size_t remaining = ctx.mmap_size - ctx.current_offset;

		if (ctx.processed_size == 0)
		{
			if (remaining < sizeof(pcap_file_header_t))
			{
                DEBUG("file too small");
				break;
			}
			ctx.current_offset += sizeof(pcap_file_header_t);
			remaining -= sizeof(pcap_file_header_t);
			ctx.bytes_processed += sizeof(pcap_file_header_t);
		}

		while (remaining >= sizeof(pcap_packet_header_t))
		{
			// 读取PCAP包头部
			pcap_packet_header_t *pcap_hdr = (pcap_packet_header_t *)(data + ctx.current_offset);

			uint32_t packet_len = pcap_hdr->incl_len;

			// 移动到包数据
			ctx.current_offset += sizeof(pcap_packet_header_t);
			remaining -= sizeof(pcap_packet_header_t);
			ctx.bytes_processed += sizeof(pcap_packet_header_t);

			// 检查是否有足够数据
			if (packet_len > remaining)
			{
				// 包不完整，回退并等待下一个内存块
				ctx.current_offset -= sizeof(pcap_packet_header_t);
				ctx.bytes_processed -= sizeof(pcap_packet_header_t);
					
				break;
			}

			// 解析以太网帧
			const uint8_t *packet_data = data + ctx.current_offset;
	
			// 跳过以太网头部 (14字节)
			if (packet_len < 14)
			{
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				ctx.bytes_processed += packet_len;
				continue;
			}

			// 检查以太网类型 (0x0800 = IPv4)
			uint16_t eth_type = ntohs(*(uint16_t *)(packet_data + 12));
			if (eth_type != 0x0800)
			{
				// 非IPv4包，跳过
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				ctx.bytes_processed += packet_len;
				continue;
			}

			// 解析IP头部
			const uint8_t *ip_header = packet_data + 14;
			size_t ip_header_len = (ip_header[0] & 0x0F) * 4; // IHL字段

			if (packet_len < 14 + ip_header_len)
			{
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				ctx.bytes_processed += packet_len;
				continue;
			}

			// 检查协议类型 (17 = UDP)
			if (ip_header[9] != 17)
			{
				// 非UDP包，跳过
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				ctx.bytes_processed += packet_len;
				continue;
			}

			// 解析UDP头部
			const uint8_t *udp_header = ip_header + ip_header_len;
			uint16_t udp_len = ntohs(*(uint16_t *)(udp_header + 4));

			// 获取UDP负载
			const uint8_t *payload = udp_header + 8;
			size_t payload_len = udp_len - 8;

			// 解析UDP负载
			bool bParser = false;
			
			if (payload_len >= 2 && payload[1] == 0xFA && (payload[0] == 0x88 || payload[0] == 0xCE || payload[0] == 0xCF || payload[0] == 0xDF || payload[0] == 0xC7 || payload[0] == 0xAA || payload[0] == 0x99))
			{
				// DEBUG("PointCloud payload_len: %lu", payload_len);
				bParser = true;
			}
			else if (payload_len >= 4 && payload[0] == 'L' && payload[1] == 'M' && payload[2] == 'S' && payload[3] == 'G')
			{
				// DEBUG("LMSG frame detected, payload_len: %lu\n", payload_len);
				bParser = true;
			}
			else if (payload_len >= 2 && payload[0] == 'A' && payload[1] == 'M')
			{
				// AM	
				bParser = true;
			}
			else if (payload_len >= 4 && payload[0] == 'L' && payload[1] == 'i' && payload[2] == 'D' && payload[3] == 'A')
			{
				// LiDA
				// DEBUG("LiDA frame detected, payload_len: %lu\n", payload_len);
			}
			else
			{
				// 其他未知负载
				// DEBUG("Unknown payload type, len=%lu, first bytes: %02X %02X %02X %02X\n", payload_len, payload[0], payload[1], payload[2], payload[3]);
			}

			if(bParser)
			{
				unsigned char *recv_data = (unsigned char *)payload;
				int recv_len = payload_len;   

				RawData *fans[MAX_FANS];
				LidarNode hP = {};
				hP.hParser = info->hParser;

				int nfan = ParserRun(hP, recv_len, recv_data, &(fans[0]));
				if (nfan > 0)
				{
					PubHub *hub = (PubHub*)info->hPublish;
					pthread_mutex_lock(&hub->mtx);
					while (hub->nfan + nfan > MAX_FANS){
						pthread_cond_wait(&hub->fans_not_full, &hub->mtx);
					}
					for (int i = 0; i < nfan; ++i)
						hub->fans[hub->nfan++] = fans[i];
					pthread_mutex_unlock(&hub->mtx);
				}
			}	

			ctx.current_offset += packet_len;
			remaining -= packet_len;
			ctx.bytes_processed += packet_len;

			/* ----- 进度输出 ----- */
			static double last_pct = -1.0;
			PRINT_PROGRESS(last_pct, ctx.bytes_processed, ctx.file_size, 0.1, (char*)info->log_file_format.c_str());
			/* -------------------- */

			usleep(10);		
		}
		ctx.processed_size = ctx.last_mapped_offset + ctx.current_offset;
	}

    DEBUG("pcap playback thread end\n");
	ctx.running = 0;
	destroy_context(&ctx);
    return NULL;
}

void* PlaybackRawThreadProc(void* p)
{
    PlaybackInfo* info = (PlaybackInfo*)p;
    ParseContext& ctx  = info->ctx;

    DEBUG("raw data file: %s", info->log_path.c_str());

    if (init_context(&ctx, info->log_path.c_str()) != 0) {
        DEBUG("init_context error for binary file");
        return nullptr;
    }

    DEBUG("file size: %.2f MB", double(ctx.file_size) / 1048576.0);

    ctx.running = 1;

    while (ctx.running && ctx.processed_size < ctx.file_size) {

        if (map_next_chunk(&ctx) != 0) {
            DEBUG("map_next_chunk failed");
            break;
        }

		DEBUG("mmap_size:%d", (int)ctx.mmap_size);

        const uint8_t* data = (const uint8_t*)ctx.mmap_ptr;
        size_t         pos  = ctx.current_offset;

        while (pos < ctx.mmap_size) {

            const size_t remaining = ctx.mmap_size - pos;

			bool bParser = false;
			size_t totalLen = 0;
		
			if (remaining >= 2 && data[pos+1] == 0xFA)
			{	
				if(data[pos] == 0x88){
					PacketUart hdr;
					memcpy(&hdr, data + pos, sizeof(PacketUart));
					totalLen = sizeof(PacketUart) + hdr.len;
				}
				else if(data[pos] == 0xCE){
					RawDataHdr hdr;
					memcpy(&hdr, data + pos, HDR_SIZE);
					totalLen = HDR_SIZE + hdr.N * 3 + 2;	//raw_bytes暂定3
				}
				else if(data[pos] == 0xCF){ //50C-R E200-A
					RawDataHdr2 hdr;
					memcpy(&hdr, data+pos, HDR2_SIZE);
					totalLen = HDR2_SIZE + hdr.N * 3 + 2;	
				}
				else if(data[pos] == 0xDF){
					RawDataHdr3 hdr;
					memcpy(&hdr, data + pos, HDR3_SIZE);
					totalLen = HDR3_SIZE + hdr.N * 3 + 2;
				}
				else if(data[pos] == 0xC7){ //E330-S
					RawDataHdr7 hdr;
					memcpy(&hdr, data+pos, HDR7_SIZE);
					totalLen = HDR7_SIZE + hdr.N * 5 + 2;
				}
				else if(data[pos] == 0xAA){
					RawDataHdrAA hdr;
					memcpy(&hdr, data + pos, HDRAA_SIZE);
					totalLen = HDRAA_SIZE + hdr.N * 3 + 2;
				}
				else if(data[pos] == 0x99){
					RawDataHdr99 hdr;
					memcpy(&hdr, data + pos, HDR99_SIZE);
					totalLen = HDR99_SIZE + hdr.N * 3 + 2;
				}
				else{

				}	

				if (totalLen > 0 && totalLen <= remaining) {
					bParser = true;
				}
			}
			else if (remaining >= sizeof(LidarAlarm) && memcmp(&data[pos], "LMSG", 4) == 0) {
				totalLen = sizeof(LidarAlarm);
				bParser  = true;
			}		
			else if (remaining >= sizeof(PROCOTOL_HOST_ALARM_ST) && memcmp(&data[pos], "AM", 2) == 0) {
				totalLen = sizeof(PROCOTOL_HOST_ALARM_ST);
				bParser  = true;
			}
			else if (remaining >= 112 && memcmp(&data[pos], "LiDA", 4) == 0) {
				totalLen = 112;                 
				pos               += totalLen;
            	ctx.bytes_processed += totalLen;
				continue;	// 预留
			}	
			
			if(!bParser)
			{
				++pos;
				++ctx.bytes_processed;
				continue;
			}
	
			RawData* fans[MAX_FANS];
			LidarNode hP = {};
			hP.hParser   = info->hParser;

			int nfan = ParserRun(hP, totalLen, (unsigned char*)(data + pos), fans);
			if (nfan > 0) {
				PubHub* hub = (PubHub*)info->hPublish;
				pthread_mutex_lock(&hub->mtx);
				while (hub->nfan + nfan > MAX_FANS)
					pthread_cond_wait(&hub->fans_not_full, &hub->mtx);
				for (int i = 0; i < nfan; ++i)
					hub->fans[hub->nfan++] = fans[i];
				pthread_mutex_unlock(&hub->mtx);
			}

            pos += totalLen;
            ctx.bytes_processed += totalLen;

			/* 进度打印 */
			static double last_pct = -1.0;
			PRINT_PROGRESS(last_pct, ctx.bytes_processed, ctx.file_size, 0.1, (char*)info->log_file_format.c_str());

			usleep(10);
        }

        ctx.current_offset = pos;
        ctx.processed_size = ctx.last_mapped_offset + ctx.current_offset;
    }

    destroy_context(&ctx);
    DEBUG("playback thread end");
    ctx.running = 0;
    return nullptr;
}

void* PlaybackHexThreadProc(void* p)
{
    PlaybackInfo* info = (PlaybackInfo*)p;
    ParseContext& ctx  = info->ctx;

    DEBUG("raw data file: %s", info->log_path.c_str());

    /* 初始化上下文 */
    if (init_context(&ctx, info->log_path.c_str()) != 0) {
        DEBUG("init_context error for binary file");
        return nullptr;
    }

    DEBUG("file size: %.2f MB", double(ctx.file_size) / 1048576.0);

	uint8_t buf[4096]; //需要大于等于帧最大长度的2倍

    ctx.running = 1;

    while (ctx.running && ctx.processed_size < ctx.file_size) {

        if (map_next_chunk(&ctx) != 0) {
            DEBUG("map_next_chunk failed");
            break;
        }

		DEBUG("mmap_size:%d", (int)ctx.mmap_size);

        const uint8_t* data = (const uint8_t*)ctx.mmap_ptr;
        size_t         pos  = ctx.current_offset;

        while (pos < ctx.mmap_size) {

            const size_t remaining = ctx.mmap_size - pos;

			bool bParser = false;
			uint16_t totalLen = 0;

			char key[8] = {0};

			for (int i = 0; i < 8; ++i) 
				key[i] = static_cast<char>(toupper(data[pos+i]));

			if (remaining >= 8 && key[2] == 'F' && key[3] == 'A')
			{	
				int datebyteslength = hex4_to_u16le((char*)(data+pos+4));

				if(key[0] == '8' && key[1] == '8'){			
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (sizeof(PacketUart) + datebyteslength)*2;
				}
				else if(key[0] == 'C' && key[1] == 'E'){
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (HDR_SIZE + datebyteslength * 3 + 2)*2; //raw_bytes暂定3
				}
				else if(key[0] == 'C' && key[1] == 'F'){ //50C-R E200-A
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (HDR2_SIZE + datebyteslength * 3 + 2)*2;
				}
				else if(key[0] == 'D' && key[1] == 'F'){
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (HDR3_SIZE + datebyteslength * 3 + 2)*2;
				}
				else if(key[0] == 'C' && key[1] == '7'){ //E330-S
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (HDR7_SIZE + datebyteslength * 5 + 2)*2;
				}
				else if(key[0] == 'A' && key[1] == 'A'){
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (HDRAA_SIZE + datebyteslength * 3 + 2)*2;
				}
				else if(key[0] == '9' && key[1] == '9'){
					if(datebyteslength<=0)
						totalLen=0;
					else	
						totalLen = (HDR99_SIZE + datebyteslength * 3 + 2)*2;
				}
				else{

				}	

				if (totalLen > 0 && totalLen <= remaining) {
					bParser = true;
				}
			}
			else if (remaining >= sizeof(LidarAlarm)*2 && key[0]=='4' && key[1]=='C' && key[2]=='4' && key[3]=='D' && key[4]=='5' && key[5]=='3' && key[6]=='4' && key[7]=='7') {
				totalLen = sizeof(LidarAlarm)*2;
				bParser  = true;
			}		
			else if (remaining >= sizeof(PROCOTOL_HOST_ALARM_ST)*2 && key[0]=='4' && key[1]=='1' && key[2]=='4' && key[3]=='D') {
				totalLen = sizeof(PROCOTOL_HOST_ALARM_ST)*2;
				bParser  = true;
			}
			else if (remaining >= 112*2 && key[0]=='4' && key[1]=='C' && key[2]=='6' && key[3]=='9' && key[4]=='4' && key[5]=='4' && key[6]=='4' && key[7]=='1') {
				totalLen = 112*2;                 
				pos               += totalLen;
            	ctx.bytes_processed += totalLen;
				continue;	//预留
			}	
			
			if(!bParser)
			{
				++pos;
				++ctx.bytes_processed;
				continue;
			}

			memset(buf, 0, sizeof(buf));

			//hex2byte
			uint16_t totalbyteslength = hex_str_to_bytes((const char *)data+pos, totalLen, buf);

			if(totalbyteslength==totalLen/2)
			{
				RawData* fans[MAX_FANS];
				LidarNode hP = {};
				hP.hParser   = info->hParser;

				int nfan = ParserRun(hP, totalbyteslength, (unsigned char*)(buf), fans);

				if (nfan > 0) {
					PubHub* hub = (PubHub*)info->hPublish;
					pthread_mutex_lock(&hub->mtx);
					while (hub->nfan + nfan > MAX_FANS)
						pthread_cond_wait(&hub->fans_not_full, &hub->mtx);
					for (int i = 0; i < nfan; ++i)
						hub->fans[hub->nfan++] = fans[i];
					pthread_mutex_unlock(&hub->mtx);
				}
			}

            pos += totalLen;
            ctx.bytes_processed += totalLen;

			/* 进度打印 */
			static double last_pct = -1.0;
			PRINT_PROGRESS(last_pct, ctx.bytes_processed, ctx.file_size, 0.1, (char*)info->log_file_format.c_str());

			usleep(10);
        }

        ctx.current_offset = pos;
        ctx.processed_size = ctx.last_mapped_offset + ctx.current_offset;
    }

    destroy_context(&ctx);
    DEBUG("playback thread end");
    ctx.running = 0;
    return nullptr;
}