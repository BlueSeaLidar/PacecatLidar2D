#include "../include/reader.h"
#include "../include/global.h"
#include <ctime>

extern "C" int change_baud(int fd, unsigned int baud);

bool uart_talk(int fd, int n, const char *cmd, int nhdr, const char *hdr_str, int nfetch, char *fetch, int waittime)
{
	DEBUG("send command : %s", cmd);
	write(fd, cmd, n);

	// waittime 为总超时(毫秒)，期间持续读串口、跳过点云数据，直到命中命令回复
	struct timeval start;
	gettimeofday(&start, NULL);
	long long deadline = (long long)start.tv_sec * 1000 + start.tv_usec / 1000 + waittime;

	// 最长匹配串长度(hdr_str/cmd/"unsupport")，滑动窗口据此保留跨块重叠
	int maxMatch = nhdr > n ? nhdr : n;
	if (maxMatch < 9)
		maxMatch = 9;

	const int CacheLength = 4096;
	unsigned char buf[CacheLength];
	int rlen = 0;     // 缓冲区内已读字节数
	int searched = 0; // 已搜索到的偏移，保留重叠避免跨块漏匹配

	while (1)
	{
		struct timeval now;
		gettimeofday(&now, NULL);
		long long now_ms = (long long)now.tv_sec * 1000 + now.tv_usec / 1000;
		if (now_ms >= deadline)
			break;

		// 缓冲区将满：丢弃已搜索过的旧数据，仅保留可能跨块匹配的重叠字节
		if (rlen >= CacheLength - 1)
		{
			int keep = maxMatch - 1;
			if (keep >= rlen)
				break; // 匹配串长度不小于缓冲区，异常退出
			memmove(buf, buf + rlen - keep, keep);
			rlen = keep;
			searched = 0;
		}

		int room = CacheLength - rlen - 1;
		if (room <= 0)
			continue;

		int nread = SystemAPI::Read(fd, buf + rlen, room);
		if (nread > 0)
		{
			rlen += nread;
		}
		else if (nread < 0)
		{
			break; // 读错误
		}
		else
		{
			// usleep(1000);
			continue; // 暂无数据
		}

		// 在 [searched, rlen) 内搜索匹配
		bool pending = false;
		for (int i = searched; i < rlen; i++)
		{
			// 1) 头部串匹配
			if (nhdr > 0 && i + nhdr <= rlen && memcmp(buf + i, hdr_str, nhdr) == 0)
			{
				// 头部已命中但结果未读完，等待更多数据后从该位置继续
				if (nfetch > 0 && i + nhdr + nfetch > rlen)
				{
					searched = i;
					pending = true;
					break;
				}
				if (nfetch > 0)
				{
					if (strcmp(cmd, "LVERSH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0 || strcmp(cmd, "LQAZNH") == 0 || strcmp(cmd, "LQPSTH") == 0 || strcmp(cmd, "LQNPNH") == 0 || strcmp(cmd, "LQOUTH") == 0 || strcmp(cmd, "LQCIRH") == 0 || strcmp(cmd, "LQFIRH") == 0 || strcmp(cmd, "LQSRPMH") == 0 || strcmp(cmd, "LQSMTH") == 0 || strcmp(cmd, "LQDSWH") == 0 || strcmp(cmd, "LQZTPH") == 0 || strcmp(cmd, "LQSAFH") == 0 || strcmp(cmd, "LQZIRH") == 0 || strcmp(cmd, "LURERH") == 0)
					{
						memcpy(fetch, buf + i + nhdr, nfetch);
						fetch[nfetch] = '\0';
					}
					else if (strstr(cmd, "LSRPM") != NULL)
					{
						if (buf[i + nhdr + 1] == 'O' && buf[i + nhdr + 2] == 'K')
						{
							strncpy(fetch, "OK", 3);
							fetch[2] = '\0';
						}
						else if (buf[i + nhdr + 1] == 'e' && buf[i + nhdr + 2] == 'r')
						{
							strncpy(fetch, "NG", 3);
							fetch[2] = '\0';
						}
					}
					else
					{
						strncpy(fetch, "OK", 3);
						fetch[2] = '\0';
					}
				}
				return true;
			}
			// 2) 命令回显匹配
			if (n > 0 && i + n <= rlen && memcmp(buf + i, cmd, n) == 0)
			{
				if (nfetch > 0 && i + n + 3 > rlen)
				{
					searched = i;
					pending = true;
					break;
				}
				if (nfetch > 0)
				{
					memcpy(fetch, buf + i + n + 1, 2);
					if (buf[i + n + 1] == 'E' && buf[i + n + 2] == 'R')
					{
						fetch[0] = 'N';
						fetch[1] = 'G';
					}
					fetch[2] = 0;
				}
				return true;
			}
			// 3) unsupport
			if (i + 9 <= rlen && memcmp(buf + i, "unsupport", 9) == 0)
			{
				if (nfetch > 0)
				{
					strcpy(fetch, "unsupport");
					fetch[10] = 0;
				}
				return true;
			}
		}

		if (pending)
			continue; // 头部已命中但结果未读完，继续读

		// 未命中，保留重叠字节继续读
		searched = rlen > (maxMatch - 1) ? rlen - (maxMatch - 1) : 0;
	}

	ERROR((int)CMD_NO_ANSWER, Error::GetErrorString(CMD_NO_ANSWER).c_str());
	return false;
}

bool vpc_talk(int hcom, int mode, short sn, int len, const char *cmd, int nfetch, void *result, int waittime)
{
	DEBUG("USB send command : %s\n", cmd);
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	write(hcom, buffer, len2);

	// waittime 为总超时(毫秒)，期间持续读串口、跳过点云数据，直到命中命令回复
	struct timeval start;
	gettimeofday(&start, NULL);
	long long deadline = (long long)start.tv_sec * 1000 + start.tv_usec / 1000 + waittime;

	// 匹配回复帧所需字节：帧头 + sn + len 字段（C_PACK/S_PACK 的 payload 都在 offset 8）
	const int CacheLength = 4096;
	const int matchLen = 8;
	unsigned char buf[CacheLength];
	int rlen = 0;     // 缓冲区内已读字节数
	int searched = 0; // 已搜索到的偏移，保留重叠避免跨块漏匹配

	while (1)
	{
		struct timeval now;
		gettimeofday(&now, NULL);
		long long now_ms = (long long)now.tv_sec * 1000 + now.tv_usec / 1000;
		if (now_ms >= deadline)
			break;

		// 缓冲区将满：丢弃已搜索过的旧数据，仅保留可能跨块匹配的重叠字节
		if (rlen >= CacheLength - 1)
		{
			int keep = matchLen - 1;
			if (keep >= rlen)
				break;
			memmove(buf, buf + rlen - keep, keep);
			rlen = keep;
			searched = 0;
		}

		int room = CacheLength - rlen - 1;
		if (room <= 0)
			continue;

		int nread = SystemAPI::Read(hcom, buf + rlen, room);
		if (nread > 0)
		{
			rlen += nread;
		}
		else if (nread < 0)
		{
			break; // 读错误
		}
		else
		{
			// usleep(1000);
			continue; // 暂无数据
		}

		// 在 [searched, rlen) 中搜索命令回复帧头
		bool pending = false;
		for (int i = searched; i + matchLen <= rlen; i++)
		{
			if (mode == C_PACK)
			{
				if (buf[i] == 0x4C && buf[i + 1] == 0x48 && buf[i + 2] == 0xBC && buf[i + 3] == 0xFF)
				{
					short packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					int payloadLen = ((int)buf[i + 7] << 8) | (int)buf[i + 6];
					if (packSN != sn)
						continue;

					int need = payloadLen < nfetch ? payloadLen : nfetch;
					if (need <= 0)
						need = nfetch;
					// 帧头命中但 payload 未读完，等待更多数据后从该位置继续
					if (i + 8 + need > rlen)
					{
						searched = i;
						pending = true;
						break;
					}

					char *fetch = (char *)result;
					for (int j = 0; j < need; j++)
					{
						if (buf[i + 8 + j] >= 33 && buf[i + 8 + j] <= 127)
							fetch[j] = buf[i + 8 + j];
						else
							fetch[j] = ' ';
					}
					fetch[need] = 0;
					return true;
				}
			}
			else if (mode == S_PACK)
			{
				if ((buf[i + 2] == 0xAC && buf[i + 3] == 0xB8) || (buf[i + 2] == 0xAC && buf[i + 3] == 0xff))
				{
					short packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					int payloadLen = ((int)buf[i + 7] << 8) | (int)buf[i + 6];
					if (packSN != sn)
						continue;

					int need = payloadLen < nfetch ? payloadLen : nfetch;
					if (need <= 0)
						need = nfetch;
					// 帧头命中但结果未读完，等待更多数据
					if (i + 8 + need > rlen)
					{
						searched = i;
						pending = true;
						break;
					}

					memcpy(result, buf + i + 8, need);
					return true;
				}
			}
		}

		if (pending)
			continue; // 帧头已命中但结果未读完，继续读

		// 未命中，保留重叠字节继续读
		searched = rlen > (matchLen - 1) ? rlen - (matchLen - 1) : 0;
	}

	ERROR((int)CMD_NO_ANSWER, Error::GetErrorString(CMD_NO_ANSWER).c_str());
	return false;
}

int open_serial_port(const char *port, unsigned int baudrate)
{
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		// DEBUG("open %s error", port);
		ERROR((int)COM_OPEN_ERR, Error::GetErrorString(COM_OPEN_ERR).c_str());
		return -1;
	}

	int ret;
	struct termios attrs;

	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		// DEBUG("get attrs failed");
		ERROR((int)GET_COM_ARG_ERR, Error::GetErrorString(GET_COM_ARG_ERR).c_str());
		return -1;
	}

	/* set speed */
	// int speed = B230400;
	//  if (baudrate == 115200) speed = B115200;

	// ret = cfsetispeed(&attrs, speed);  //[baudrate]);
	// ret |= cfsetospeed(&attrs, speed); //[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1)
	{							  // parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB; // disable parity
		attrs.c_iflag &= ~INPCK;
	}
	else
	{
		attrs.c_cflag |= (PARENB | PARODD); // enable parity
		attrs.c_iflag |= INPCK;
		// if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB; // 1 stop bit
							  // attrs.c_cflag |= CSTOPB;	// 2 stop bits

	// Disable Hardware flowcontrol
	attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

	attrs.c_cc[VMIN] = 0;
	attrs.c_cc[VTIME] = 0;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if (tcsetattr(fd, TCSANOW, &attrs) < 0)
	{
		close(fd);
		// DEBUG("tcsetattr err");
		ERROR((int)SET_COM_ARG_ERR, Error::GetErrorString(SET_COM_ARG_ERR).c_str());
		return -1;
	}

	if (change_baud(fd, baudrate))
	{
		close(fd);
		// DEBUG("fail to set baudrate %d", baudrate);
		ERROR((int)CHANGE_BAUDRATE_ERR, Error::GetErrorString(CHANGE_BAUDRATE_ERR).c_str());
		return -1;
	}

	return fd;
}

bool setup_lidar_uart(Parser *hP, int handle)
{
	Parser *parser = (Parser *)hP;
	unsigned int index = 5;
	int cmdLength;
	char buf[32];

	for (unsigned int i = 0; i < index; i++)
	{
		if ((uart_talk(handle, 6, "LVERSH", 14, "MOTOR VERSION:", 16, buf)))
		{
			std::string sn = stringfilter(buf, 16);
			DEBUG("version:%s\n", sn.c_str());
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.uuid);
		if (cmdLength <= 0)
			break;
		if ((uart_talk(handle, 6, "LUUIDH", 11, "PRODUCT SN:", 20, buf)))
		{
			std::string sn = stringfilter(buf, 20);
			DEBUG("uuid:%s\n", sn.c_str());
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.model);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.model, 8, "TYPE ID:", 16, buf))
		{
			std::string sn = stringfilter(buf, 16);
			DEBUG("product model:%s\n", sn.c_str());
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.unit_mm);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.unit_mm, 10, "SET LiDAR ", 12, buf))
		{
			DEBUG("set LiDAR unit %s\n", buf);
			break;
		}
	}
	// enable/disable output intensity
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.confidence);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.confidence, 6, "LiDAR ", 12, buf))
		{
			DEBUG("set LiDAR confidence %s\n", buf);
			break;
		}
	}
	// enable/disable shadow filter
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.fitter);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.fitter, 2, "ok", 2, buf, 100))
		{
			DEBUG("set LiDAR shadow filter ok\n");
			break;
		}
	}
	// enable/disable smooth

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.smooth);
		if (cmdLength <= 0)
			break;

		if (uart_talk(handle, cmdLength, parser->cmd.smooth, 2, "ok", 2, buf, 100))
		{
			DEBUG("set LiDAR smooth OK\n");
			break;
		}
	}
	// 设置旋转方向
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.direction);
		if (cmdLength <= 0)
			break;

		if (uart_talk(handle, cmdLength, parser->cmd.direction, 2, "OK", 2, buf, 100))
		{
			DEBUG("set LiDAR direction OK\n");
			break;
		}
	}

	// setup rpm
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.rpm);
		if (cmdLength <= 0)
			break;

		if (uart_talk(handle, cmdLength, parser->cmd.rpm, 8, "Set RPM:", 12, buf, 100))
		{
			DEBUG("%s %s\n", parser->cmd.rpm, buf);
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.res);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.res, 15, "set resolution ", 12, buf))
		{
			DEBUG("%s %s\n", parser->cmd.res, buf);
			break;
		}
	}
	return true;
}

bool setup_lidar_vpc(Parser *hP, int handle)
{
	Parser *parser = (Parser *)hP;
	unsigned int index = 5;
	int cmdLength;
	char buf[32];
	char result[3] = {0};
	result[2] = '\0';

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.ats);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.ats, 2, result))
		{
			DEBUG("%s %s\n", parser->cmd.ats, result);
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.uuid);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0043, rand(), cmdLength, parser->cmd.uuid, 20, buf))
		{
			std::string sn = stringfilter(buf, 20);
			DEBUG("uuid:%s\n", sn.c_str());
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.model);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0043, rand(), cmdLength, parser->cmd.model, 16, buf))
		{
			std::string sn = stringfilter(buf, 16);
			DEBUG("product model:%s\n", sn.c_str());
			break;
		}
	}

	// enable/disable shadow filter
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.fitter);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.fitter, 2, result))
		{
			DEBUG("set LiDAR shadow filter %s %s\n", parser->cmd.fitter, result);
			break;
		}
	}
	// enable/disable smooth
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.smooth);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.smooth, 2, result))
		{
			DEBUG("set LiDAR smooth  %s %s\n", parser->cmd.smooth, result);
			break;
		}
	}
	// setup rpm
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.rpm);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.rpm, 2, result))
		{
			DEBUG("%s %s\n", parser->cmd.rpm, result);
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.res);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.res, 2, result))
		{
			DEBUG("%s %s\n", parser->cmd.res, result);
			break;
		}
	}

	// enable/disable alaram message uploading
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.alarm);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.alarm, 2, result))
		{
			DEBUG("set alarm_msg %s\n", result);
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.direction);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.direction, 2, result))
		{
			DEBUG("set direction %s\n", result);
			break;
		}
	}
	return true;
}

int UartReader(UartInfo *info)
{
	int fd_uart = info->fd_uart;
	RawData *fans[MAX_FANS];

	timeval tv;
	gettimeofday(&tv, NULL);
	time_t tto = tv.tv_sec + 1;

	info->nodata_count = 0;

	while (1)
	{
		if (info->client_cmd_pause)
		{
			usleep(50000);
			continue;
		}

		gettimeofday(&tv, NULL);
		if (tv.tv_sec > tto)
		{
			if(access(info->port, F_OK))
			{
				DEBUG("uart %s not found\n", info->port);
				//test
				// strcpy(info->port, "/dev/ttyUSB1");
				break;
			}

			tto = tv.tv_sec + 1;
			if (info->hasdata == true)
			{
				info->hasdata = false;
			}
			else
			{
				info->nodata_count++;
				DEBUG("lidar data is not recv,count:%d\n", info->nodata_count);

				unsigned int index = 5;
				for (unsigned int i = 0; i < index; i++)
				{
					char buf[32];
					if (strcmp(info->type, "uart") == 0)
					{	
						if (uart_talk(fd_uart, 6, "LUUIDH", 11, "PRODUCT SN:", 20, buf))
						{
							std::string sn = stringfilter(buf, 20);
							DEBUG("recv uuid:%s\n", sn.c_str());
							break;
						}
					}
					else if (strcmp(info->type, "vpc") == 0)
					{
						if (vpc_talk(fd_uart, 0x0043, rand(), 6, "LUUIDH", 20, buf))
						{
							std::string sn = stringfilter(buf, 20);
							DEBUG("recv uuid:%s\n", sn.c_str());
							break;
						}
					}
				}	
			}	
		}

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_uart, &fds);
		struct timeval to = {1, 0};
		int ret = select(fd_uart + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			// printf("select error\n");
			ERROR((int)SOCKET_ERR, Error::GetErrorString(SOCKET_ERR).c_str());
			break;
		}

		// read UART data
		if (FD_ISSET(fd_uart, &fds))
		{
			if (info->client_cmd_pause)
			{
				usleep(50000);
				continue;
			}
			unsigned char buf[512];
			int nr = SystemAPI::Read(fd_uart, buf, sizeof(buf));
			if (nr < 0)
			{
				// printf("read port error %d\n", nr);
				ERROR((int)COM_READ_ERR, Error::GetErrorString(COM_READ_ERR).c_str());
				break;
			}
			if (nr == 0)
			{
				// printf("read nr==0, %d\n", errno);
				continue;
			}

			{// record UART data
				static FILE *record_file = NULL;
				static int last_record_state = 0;
				static char record_filename[128] = {0};
				if (g_uart_record_enabled != last_record_state)
				{
					if (g_uart_record_enabled)
					{
						time_t now = time(NULL);
						struct tm tm_buf;
						localtime_r(&now, &tm_buf);
						snprintf(record_filename, sizeof(record_filename),
							"/tmp/uart_record_%04d%02d%02d_%02d%02d%02d.bin",
							tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
							tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
						record_file = fopen(record_filename, "wb");
						if (record_file)
							DEBUG("UART recording started, saving to %s", record_filename);
					}
					else
					{
						if (record_file)
						{
							fclose(record_file);
							record_file = NULL;
							DEBUG("UART recording stopped");
						}
					}
					last_record_state = g_uart_record_enabled;
				}
				if (g_uart_record_enabled && record_file)
				{
					fwrite(buf, 1, nr, record_file);
					fflush(record_file);
				}
			}

			int nfan = ParserRunStream(info->hParser, nr, buf, &(fans[0]));
			if (nfan > 0)
			{
				PublishData(info->hPublish, nfan, fans);
				info->hasdata = true;
				info->nodata_count = 0;
			}
			// if (nr < sizeof(buf) - 10)
			// 	usleep(10000 - nr * 10);
		}
	}
	return 0;
}

void *UartThreadProc(void *p)
{
	while (1)
	{
		UartInfo *info = (UartInfo *)p;
		if (access(info->port, R_OK))
		{
			ERROR((int)DEVICE_OFFLINE, Error::GetErrorString(DEVICE_OFFLINE).c_str());
			sleep(3);
			continue;
		}
		unsigned int baudrate = 0;
		if (info->baudrate <= 0)
		{
			baudrate = GetComBaud(info->port, info->hParser);
		}
		else
			baudrate = info->baudrate;

		DEBUG("Connection info:uartname %s baudrate %d", info->port, baudrate);

		if(baudrate<=0)
		{
			DEBUG("Auto baud rate detection failed. Please fill in the serial port baud rate in the configuration file.");
			return NULL;
		}

		int fd = open_serial_port(info->port, baudrate);	
		if (fd > 0)
		{
			info->fd_uart = fd;
			if (strcmp(info->type, "uart") == 0)
				setup_lidar_uart(info->hParser, info->fd_uart);
			if (strcmp(info->type, "vpc") == 0)
				setup_lidar_vpc(info->hParser, info->fd_uart);

			UartReader(info);	
		}
	}
	return NULL;
}

void *StartUartReader(const char *type, const char *port, int baudrate, Parser *hParser, PubHub *hPublish)
{
	UartInfo *info = new UartInfo;
	info->client_cmd_pause = false;
	strcpy(info->type, type);
	strcpy(info->port, port);
	info->baudrate = baudrate;
	info->hParser = hParser;
	info->hPublish = hPublish;
	pthread_create(&info->thr, NULL, UartThreadProc, info);

	return info;
}

static void *urerUartThread(void *arg)
{
	UartInfo *info = (UartInfo *)arg;

	cmd_log("========== URER UART Diagnostic Start ==========");

	for (int iter = 0; iter < 10; )
	{
		if (g_urer_restart)
		{
			g_urer_restart = false;
			iter = 0;
			cmd_log("========== URER UART Diagnostic Restart ==========");
			continue;
		}

		info->client_cmd_pause = true;
		bool ok = false;
		char buf[128] = {0};

		// 直接发送 LURERH，读取串口回复
		usleep(10000); // 等点云线程真正暂停后再发送
		tcflush(info->fd_uart, TCIFLUSH);
		write(info->fd_uart, "LURERH", 6);
		char rbuf[4096] = {0};
		int rlen = 0;
		int ridx = 50; // 50ms 超时
		while (rlen < (int)sizeof(rbuf) - 1)
		{
			int n = SystemAPI::Read(info->fd_uart, rbuf + rlen, sizeof(rbuf) - 1 - rlen);
			if (n > 0)
			{
				rlen += n;
				ridx = 50;
			}
			else if (n == 0)
			{
				ridx--;
				usleep(1000);
				if (ridx == 0)
					break;
			}
		}
		rbuf[rlen] = '\0';

		// 点云数据里含 0x00，strstr 会被截断，改用 memcmp 按字节搜索
		static const char crlf[] = "\r\n";
		static const char ok_hdr[] = "LURERH OK";
		const int crlf_len = (int)sizeof(crlf) - 1;         // 2
		const int ok_hdr_len = (int)sizeof(ok_hdr) - 1;     // 9

		// 头部用 \r\n 定位，不同雷达的字段名可能不同，不依赖 "Head:"
		char *head = NULL;
		for (int i = 0; i <= rlen - crlf_len; i++)
		{
			if (memcmp(rbuf + i, crlf, crlf_len) == 0)
			{
				head = rbuf + i + crlf_len; // 从第一个 \r\n 之后开始
				break;
			}
		}

		char *okmark = NULL;
		int start = head ? (int)(head - rbuf) : 0;
		for (int i = start; i <= rlen - ok_hdr_len; i++)
		{
			if (memcmp(rbuf + i, ok_hdr, ok_hdr_len) == 0)
			{
				okmark = rbuf + i;
				break;
			}
		}

		if (okmark)
		{
			ok = true;
			char *text = head ? head : okmark;
			int len = (int)(okmark + ok_hdr_len - text);
			if (len > 0 && len < (int)sizeof(buf))
			{
				memcpy(buf, text, len);
				buf[len] = '\0';
			}
		}

		info->client_cmd_pause = false;

		if (ok)
			cmd_log("URER UART [%d/10] OK\n%s", iter + 1, buf);
		else
			cmd_log("URER UART [%d/10] NG", iter + 1);

		iter++;
		if (iter < 10) sleep(1);
	}

	cmd_log("========== URER UART Diagnostic End ==========");
	g_urer_running = false;
	return NULL;
}

static void *urerVpcThread(void *arg)
{
	UartInfo *info = (UartInfo *)arg;

	cmd_log("========== URER VPC Diagnostic Start ==========");

	for (int iter = 0; iter < 10; )
	{
		if (g_urer_restart)
		{
			g_urer_restart = false;
			iter = 0;
			cmd_log("========== URER VPC Diagnostic Restart ==========");
			continue;
		}

		info->client_cmd_pause = true;
		bool ok = false;
		char buf[256] = {0};

		// 直接组 C_PACK 帧发送 LURERH，读取串口回复
		usleep(10000); // 等点云线程真正暂停后再发送
		tcflush(info->fd_uart, TCIFLUSH);

		char frame[2048];
		CmdHeader *hdr = (CmdHeader *)frame;
		hdr->sign = 0x484c;
		hdr->cmd = 0x0043; // C_PACK
		hdr->sn = rand();
		int flen = ((6 + 3) >> 2) * 4;
		hdr->len = flen;
		memset(frame + sizeof(CmdHeader), 0, flen);
		memcpy(frame + sizeof(CmdHeader), "LURERH", 6);
		unsigned int *pcrc = (unsigned int *)(frame + sizeof(CmdHeader) + flen);
		pcrc[0] = stm32crc((unsigned int *)frame, flen / 4 + 2);
		write(info->fd_uart, frame, flen + sizeof(CmdHeader) + 4);

		unsigned char rbuf[4096] = {0};
		int rlen = 0;
		int ridx = 50; // 50ms 超时
		while (rlen < (int)sizeof(rbuf) - 1)
		{
			int n = SystemAPI::Read(info->fd_uart, rbuf + rlen, sizeof(rbuf) - 1 - rlen);
			if (n > 0)
			{
				rlen += n;
				ridx = 50;
			}
			else if (n == 0)
			{
				ridx--;
				usleep(1000);
				if (ridx == 0)
					break;
			}
			else
				break;
		}

		// 找 C_PACK 回复头 4C 48 BC FF，解析 payload 中的 UART_STA:/UART ERR:
		// 帧结构: 4C 48 BC FF | sn(2) | len(2) | payload(len) | crc(4)
		static const unsigned char vpc_hdr[] = {0x4C, 0x48, 0xBC, 0xFF};
		for (int i = 0; i + 8 < rlen; i++)
		{
			if (memcmp(rbuf + i, vpc_hdr, 4) != 0)
				continue;
			int paylen = rbuf[i + 6] | (rbuf[i + 7] << 8);
			if (paylen > rlen - (i + 8))
				paylen = rlen - (i + 8);
			if (ExtractUartStatusLine((const char *)(rbuf + i + 8), paylen, buf, sizeof(buf)))
				ok = true;
			break;
		}

		info->client_cmd_pause = false;

		if (ok)
			cmd_log("URER VPC [%d/10] OK\n%s", iter + 1, buf);
		else
			cmd_log("URER VPC [%d/10] NG", iter + 1);

		iter++;
		if (iter < 10) sleep(1);
	}

	cmd_log("========== URER VPC Diagnostic End ==========");
	g_urer_running = false;
	return NULL;
}

bool SendUartCmd(HReader hr, int len, char *cmd)
{
	UartInfo *info = (UartInfo *)hr;
	if (!info || info->fd_uart <= 0)
		return false;

	if (strcmp(cmd, "LURERH") == 0)
	{
		if (g_urer_running)
		{
			g_urer_restart = true;
			cmd_log("URER UART -> RESTART diagnostic");
			return true;
		}
		g_urer_running = true;
		pthread_t tid;
		pthread_create(&tid, NULL, urerUartThread, info);
		pthread_detach(tid);
		cmd_log("URER UART -> STARTED diagnostic (10s, 1s interval)");
		return true;
	}

	bool ret = false;
	info->client_cmd_pause = true;
	usleep(10000); // 等点云线程真正暂停后再发送
	for (unsigned int i = 0; i < 5; i++)
	{
		tcflush(info->fd_uart, TCIFLUSH);
		int wt = (strcmp(cmd, "LSTARH") == 0) ? 5000 : 1000;
		if (uart_talk(info->fd_uart, len, cmd, 2, "OK", 0, NULL, wt))
		{
			ret = true;
			break;
		}
	}
	info->client_cmd_pause = false;
	DEBUG("UART send cmd %s %s", cmd, ret ? "OK" : "NG");
	return ret;
}

bool SendVpcCmd(HReader hr, int len, char *cmd)
{
	UartInfo *info = (UartInfo *)hr;
	if (!info || info->fd_uart <= 0)
		return false;

	if (strcmp(cmd, "LURERH") == 0)
	{
		if (g_urer_running)
		{
			g_urer_restart = true;
			cmd_log("URER VPC -> RESTART diagnostic");
			return true;
		}
		g_urer_running = true;
		pthread_t tid;
		pthread_create(&tid, NULL, urerVpcThread, info);
		pthread_detach(tid);
		cmd_log("URER VPC -> STARTED diagnostic (10s, 1s interval)");
		return true;
	}

	bool ret = false;
	info->client_cmd_pause = true;
	usleep(10000); // 等点云线程真正暂停后再发送
	int wt = (strcmp(cmd, "LSTARH") == 0) ? 5000 : 1000;
	for (unsigned int i = 0; i < 5; i++)
	{
		tcflush(info->fd_uart, TCIFLUSH);
		char buf[128] = {0};
		if (vpc_talk(info->fd_uart, 0x0043, rand(), len, cmd, 100, buf, wt))
		{
			if (strstr(buf, "OK"))
			{
				ret = true;
				break;
			}
		}
	}
	info->client_cmd_pause = false;
	DEBUG("VPC send cmd %s %s", cmd, ret ? "OK" : "NG");
	return ret;
}

void StopUartReader(HReader hr)
{
	UartInfo *info = (UartInfo *)hr;
	// info->should_exit = true;
	// sleep(1);
	pthread_join(info->thr, NULL);
	delete info;
}

int GetDevInfoByUART(const char *port_str, unsigned int speed, unsigned int check_size, unsigned int time_out, Parser* hParser)
{
    printf("try adapt:%s %d %d %d\n", port_str, speed, check_size, time_out);
 
    int hPort = open_serial_port(port_str, speed);
    if (hPort <= 0)
    {
        return false;
    }

    tcflush(hPort, TCIFLUSH);

    char cmd[] = "LVERSH";
    write(hPort, cmd, sizeof(cmd));
    int bOK = false;
    unsigned char *buf = new unsigned char[check_size];
    unsigned long rf = 0;
    
    const int HDR_LEN = 12;
    const char* HDR_STR = "MCU VERSION:";
    
    uint64_t start_time = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);

    while (rf < check_size && !bOK)
    {
        uint64_t now = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);
        if (now - start_time > time_out)
        {
            printf("timeout %lums, rf=%lu\n", now - start_time, rf);
            break;
        }

        int tmp = SystemAPI::Read(hPort, buf + rf, check_size - rf);
        if (tmp > 0)
        {
            rf += tmp;

			if (rf >= HDR_LEN && !bOK)
			{
				for (unsigned int idx = 0; idx <= rf - HDR_LEN; idx++)
				{
					if (memcmp((char *)buf + idx, HDR_STR, HDR_LEN) == 0)
					{
						printf("OK (MCU VERSION found at %u)\n", idx);
						bOK = 1;
						break;
					}
				}
			}

			// if (hParser && rf > 2 && !bOK)
			// {
			// 	if (FindValidPacket(buf, rf, hParser))
			// 	{
			// 		printf("OK (valid packet found in early check)\n");
			// 		bOK = 1;
			// 		break;
			// 	}
			// }
        }
    }

    printf("read max byte %ld break\n", rf);
    
	if (!bOK && rf > HDR_LEN)
    {
        for (unsigned int idx = 0; idx < rf - HDR_LEN; idx++)
        {
            if (memcmp((char *)buf + idx, "MCU VERSION:", HDR_LEN) == 0)
            {
                printf("OK (MCU VERSION found in final check)\n");
                bOK = 1;
                break;
            }
        }
    }

    if (!bOK && rf > 2 && hParser)
    {
        if (FindValidPacket(buf, rf, hParser))
        {
            printf("OK (valid packet found in final check)\n");
            bOK = 1;
        }
    }
     
    close(hPort);
    delete[] buf;
    return bOK;
}

bool IsLidarSpinning(const char *port_str)
{
    const unsigned int CHECK_SPEED = 230400;
    const unsigned int THRESHOLD = 128;
    
    printf("check spinning:%s %d\n", port_str, CHECK_SPEED);
    
    int hPort = open_serial_port(port_str, CHECK_SPEED);
    if (hPort <= 0) 
		return false;
    
    tcflush(hPort, TCIFLUSH);
    usleep(50000);

    const unsigned int check_size = 1024;
    unsigned char *buf = new unsigned char[check_size];
    unsigned long rf = 0;
    bool isSpinning = false;
    
    uint64_t start_time = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);
    
    while (rf < THRESHOLD)
    {
        uint64_t now = SystemAPI::getTimestamp(TimeUnit::MILLISECOND);
        if (now - start_time > 200)
        {
			printf("timeout %lums, rf=%lu\n", now - start_time, rf);
			break;
		}
        
        int tmp = SystemAPI::Read(hPort, buf + rf, check_size - rf);
        if (tmp > 0)
        {
            rf += tmp;
        }

		if (rf >= THRESHOLD)
        {
			printf("threshold reached, rf=%lu\n", rf);
            isSpinning = true;
            break;
        }
    }
    
    printf("Lidar %s: %lu bytes\n", isSpinning ? "spinning" : "stopped", rf);
    
    close(hPort);
    delete[] buf;
    return isSpinning;
}

int GetComBaud(std::string uartname, Parser* hParser)
{
	std::vector<int> port_list;

	port_list.push_back(230400);
	port_list.push_back(256000);
	port_list.push_back(384000);
	port_list.push_back(460800);
	port_list.push_back(500000);
	port_list.push_back(768000);
	port_list.push_back(921600);
	port_list.push_back(1000000);
	port_list.push_back(1500000);

	unsigned int check_size = 4096;
	unsigned int time_out;

	if(IsLidarSpinning(uartname.c_str()))
	{
		time_out = 1000;
	}
	else
	{
		time_out = 200;
	}

	for (unsigned int j = 0; j < port_list.size(); j++)
	{
		unsigned int com_speed = port_list.at(j);
		if (GetDevInfoByUART(uartname.c_str(), com_speed, check_size, time_out, hParser))
		{
			return com_speed;
		}
	}
	return 0;
}

