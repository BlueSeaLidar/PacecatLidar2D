#include <termios.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include<sys/time.h>
#include<unistd.h>
#include<fcntl.h>
//#include <windows.h>
//#include <setupapi.h>
//#include <winioctl.h>
#define  ULONGLONG  unsigned long long

ULONGLONG GetTickCount64()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	unsigned long long  t = tv.tv_sec*1000+tv.tv_usec/1000;//ms
	return t;
}
extern "C"  int change_baud(int fd, int baud);
extern "C"
int xmodem_send(int serial_fd, size_t len, const uint8_t * buf);

extern "C"
int read_char(int fd, uint8_t* buf, int timeout)
{
	
	ULONGLONG t = GetTickCount64();
	while (GetTickCount64() < t + timeout)
	{
		int dw = read(fd, buf, 1);
		if (dw)
		{
			return 1;
		}
	}
	return -1;
}

extern "C"
int xread(int fd, void* buf, size_t sz)
{
	size_t nr = 0;
	char* buff = (char*)buf;
	while (nr < sz)
	{
		unsigned long dw=read(fd, buf, sz - nr);
		if (dw)
			nr += dw;
		else
			usleep (1000);
	}
	return nr;
}

extern "C"
int xwrite(int fd, void* buf, size_t sz)
{
	return write(fd, buf, sz);
}
int OpenPort(const char* name, int speed, int stop_bit)
{

	int fd = open(name, O_RDWR | O_NOCTTY | O_NDELAY);
	// INFO_PR("open_serial_port %d\n",fd);
	if (fd < 0)
	{
		printf("\033[1;31m----> Open %s error\033[0m\n", name);
		return -1;
	}

	int ret;
	struct termios attrs;
	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		printf("get attrs failed\n");
		return -1;
	}

	/* set speed */
	int speed_tmp = B230400;
	// if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed_tmp);  //[baudrate]);
	ret |= cfsetospeed(&attrs, speed_tmp); //[baudrate]);

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
	if(stop_bit==2)
		attrs.c_cflag |= CSTOPB;
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

	//	attrs.c_cc[VMIN] = 0;
	//	attrs.c_cc[VTIME] = 10;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if (tcsetattr(fd, TCSANOW, &attrs) < 0)
	{
		close(fd);
		printf("tcsetattr err");
		return -1;
	}

	if (change_baud(fd, speed))
	{
		close(fd);
		printf("fail to set baudrate %d", speed);
		return -1;
	}

	return fd;
}
#include <stdio.h>
#include <string.h>
#include <errno.h>
uint8_t* load_bin(const char* path, size_t& len)
{
	FILE* fp = fopen(path, "rb");
	if (fp== NULL) {
		printf("open fail errno = %d reason = %s \n", errno, strerror(errno));
		return NULL;
	}
	uint8_t* file_buf = new uint8_t[1024 * 1024];
	len = fread(file_buf, 1, 1024 * 1024, fp);
	fclose(fp);

	return file_buf;
}

int XReadFile(int h, char* line, int sz)
{
	int nr = 0;
	int  dw;
	ULONGLONG t = GetTickCount64();
	while (GetTickCount64() < t+200 && nr < sz) 
	{
		dw = read(h, line + nr, sz - nr);
		if (dw>0)
		{
			nr += dw;
		}
	}
	return nr;
}

// Extract the version string after the keyword from the response text (truncated at \r or \n)
static int extract_version(const char* buf, const char* key, char* out, int out_sz)
{
	const char* p = strstr(buf, key);
	if (!p)
		return -1;
	p += strlen(key);
	while (*p == ' ' || *p == '\t')   // skip spaces after the colon
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

// Query version: send LVERSH and parse MCU VERSION and MOTOR VERSION
static void query_version(int h)
{
	char buf[8192];
	int nr = 0;
	ULONGLONG t0 = GetTickCount64();       // overall timeout start
	ULONGLONG tlast = t0;                  // last send time

	while (GetTickCount64() < t0 + 1000 && nr < (int)sizeof(buf) - 1)
	{
		if (GetTickCount64() > tlast + 100) // resend every 100ms (to handle device restart after upgrade)
		{
			write(h, "LVERSH", 6);
			tlast = GetTickCount64();
			//printf("[%llums] send LVERSH\n", tlast - t0);
		}
		int r = XReadFile(h, buf + nr, (int)(sizeof(buf) - 1 - nr));
		if (r > 0)
		{
			nr += r;
			buf[nr] = '\0';
			//printf("[%llums] recv %d bytes, total %d\n", GetTickCount64() - t0, r, nr);
		}
		// finish early once both version strings are received
		if (strstr(buf, "MCU VERSION:") && strstr(buf, "MOTOR VERSION:"))
		{
			//printf("[%llums] got both versions\n", GetTickCount64() - t0);
			break;
		}
	}
	buf[nr] = '\0';

	char head[64] = {0};
	char board[64] = {0};

	if (extract_version(buf, "MCU VERSION:", head, sizeof(head)) == 0)
		printf("MCU VERSION: %s\n", head);
	else
		printf("MCU VERSION: unknown\n");

	if (extract_version(buf, "MOTOR VERSION:", board, sizeof(board)) == 0)
		printf("MOTOR VERSION: %s\n", board);
	else
		printf("MOTOR VERSION: unknown\n");
}

int main(int argc, char* argv[])
{
	if (argc < 4) {
		printf("usage : %s comX baudrate bin-file-path\n", argv[0]);
		return -1;
	}

	size_t len = 0;
	uint8_t* file = load_bin(argv[3], len);
	if (!file)
	{
		printf("can not load bin : %s\n", argv[3]);
		return -1;
	}
	

	int h = OpenPort(argv[1], atoi(argv[2]), 1);
	if (h <=0) {
		printf("can not open %s\n", argv[1]);
		return -2;
	}
	char line[1025];
	unsigned long dw;
	ULONGLONG t = GetTickCount64();
	bool bReady = false;
	while (!bReady) {
		if (GetTickCount64() > t + 500)
		{
			printf("send : LFWUPH\n");
			dw = write(h, "LFWUPH", 6);
			t = GetTickCount64();
		}
		dw = XReadFile(h, line, 1024);
		if (dw > 0) 
		{
			line[dw] = 0;
			printf("lidar : %s\n", line);
			for (int i = 0; i + 6 <= dw; i++) {
				if (memcmp(line + i, "LFWUQH", 6) == 0)
				{
					bReady = true;
					break;
				}
			}
		}
	}
	int nl = sprintf(line,"LFWUI:%08lX,%08X,H", len, 0xffffffff);
	dw = write(h, line, nl);
	t = GetTickCount64();
	bReady = false;
	int nr = 0;
	while (!bReady && nr < 1000)
	{
		int r = XReadFile(h, line+nr, 1024-nr);
		
		if (r > 0)
		{
			line[nr+r] = 0;
			printf("lidar : %s\n", line+nr);
			nr += r;
			for (int i = 0; i + 7 <= nr; i++) {
				if (memcmp(line + i, "FWUI OK", 7) == 0)
				{
					bReady = true;
					break;
				}
			}
		}
		if (GetTickCount64() > t + 2000) break;
	} 
	
	if (!bReady) {
		printf("lidar did not ack\n");
		return -4;
	}
	//printf("%d %s %d %d\n",__LINE__,__FUNCTION__,len,file);
	if (xmodem_send(h, len, file) != 0)
	{
		printf("transfer file errror\n");
		return -5;
	}

	t = GetTickCount64();
	nr = 0;
	bool bSuccess = false;
	while ( GetTickCount64() < t + 2000 && nr < 1000) 
	{
		int r = XReadFile(h, line+nr, 1024-nr);
		if (r > 0)
		{
			line[nr+r] = 0;
			//printf("lidar : %s\n", line+nr);
			nr += r;
			for (int i = 0; i + 6 <= nr; i++) {
				if (memcmp(line + i, "LFWUOH", 6) == 0)
				{
					printf("upgrade success\n");
					bSuccess = true;
					bReady = true;
					break;
				}
				if (memcmp(line + i, "LFWUNH", 6) == 0)
				{
					printf("upgrade fail!\n");
					bReady = true;
					break;
				}
			}
		}
	} 

	if (bSuccess)
		query_version(h);
}
