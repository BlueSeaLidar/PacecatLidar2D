#include "../include/global.h"
#include <stdio.h>
#include <pthread.h>
#include <sys/stat.h>

int g_uart_record_enabled = 0;

// command log
std::string g_cmd_log_path;
static pthread_mutex_t g_cmd_log_mtx = PTHREAD_MUTEX_INITIALIZER;

// URER diagnostic state
volatile bool g_urer_running = false;
volatile bool g_urer_restart = false;

void cmd_log(const char *fmt, ...)
{
	pthread_mutex_lock(&g_cmd_log_mtx);

	// lazy init: create log folder on first call
	static bool s_log_init = false;
	if (!s_log_init)
	{
		s_log_init = true;
		// PWD usually preserves the directory where roslaunch was invoked
		const char *dir = getenv("PWD");
		if (!dir || dir[0] == '\0')
		{
			char cwd[512];
			dir = getcwd(cwd, sizeof(cwd)) ? cwd : ".";
		}
		char dirpath[768];
		snprintf(dirpath, sizeof(dirpath), "%s/pacecat_log", dir);
		mkdir(dirpath, 0755);
		timeval tv;
		gettimeofday(&tv, NULL);
		tm t;
		localtime_r(&tv.tv_sec, &t);
		char filepath[768];
		snprintf(filepath, sizeof(filepath), "%s/pacecat_log/pacecat_%04d%02d%02d_%02d%02d%02d.log",
			dir,
			t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
		g_cmd_log_path = filepath;
	}

	FILE *f = fopen(g_cmd_log_path.c_str(), "a");
	if (f)
	{
		timeval tv;
		gettimeofday(&tv, NULL);
		tm t;
		localtime_r(&tv.tv_sec, &t);
		fprintf(f, "[%04d-%02d-%02d %02d:%02d:%02d.%03d] ",
				t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
				t.tm_hour, t.tm_min, t.tm_sec, (int)(tv.tv_usec / 1000));
		va_list ap;
		va_start(ap, fmt);
		vfprintf(f, fmt, ap);
		va_end(ap);
		fprintf(f, "\n");
		fclose(f);
	}
	pthread_mutex_unlock(&g_cmd_log_mtx);
}

// 从 C_PACK payload 中提取 UART_STA:/UART ERR: 诊断状态行
bool ExtractUartStatusLine(const char *payload, int len, char *out, int outlen)
{
	const char *marker = NULL;
	for (int i = 0; i + 9 <= len; i++)
	{
		if (memcmp(payload + i, "UART_STA:", 9) == 0 ||
			memcmp(payload + i, "UART ERR:", 9) == 0)
		{
			marker = payload + i;
			break;
		}
	}
	if (!marker)
		return false;

	int i = 0;
	while (i < outlen - 1 && marker + i < payload + len)
	{
		char c = marker[i];
		if (c == '\r' || c == '\n' || c == '\0')
			break;
		out[i++] = c;
	}
	// 去掉尾部的空格/填充
	while (i > 0 && out[i - 1] == ' ')
		i--;
	out[i] = '\0';
	return i > 0;
}

std::string BaseAPI::stringfilter(char *str, int num)
{
	int index = 0;
	for (int i = 0; i < num; i++)
	{
		if ((str[i] >= 45 && str[i] <= 58) || (str[i] >= 65 && str[i] <= 90) || (str[i] >= 97 && str[i] <= 122) || str[i] == 32 || str[i] == '_')
		{
			index++;
		}
		else
		{
			std::string arr = str;
			arr = arr.substr(0, index);
			return arr;
		}
	}
	return "";
}

int SystemAPI::getLastError()
{
	return errno;
}

int SystemAPI::closefd(int __fd)
{
	shutdown(__fd, SHUT_RDWR);
	return close(__fd);
}

in_addr_t SystemAPI::get_interface_ip(const char *ifname)
{
	if (!ifname || strlen(ifname) == 0)
	{
		return INADDR_NONE;
	}

	// Linux/Unix 实现 (保持不变)
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
	{
		perror("socket");
		return INADDR_NONE;
	}

	struct ifreq ifr;
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';

	if (ioctl(fd, SIOCGIFADDR, &ifr) < 0)
	{
		perror("ioctl(SIOCGIFADDR)");
		close(fd);
		return INADDR_NONE;
	}

	close(fd);
	return ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
}

uint64_t SystemAPI::GetTimeStampNow()
{
	timeval tv;
	gettimeofday(&tv, NULL);
	uint64_t ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    return ms;
}

// 返回格式：YYYY-MM-DD HH:MM:SS.mmm
std::string SystemAPI::GetTimeYMDHMSms()
{
    timeval tv;
    gettimeofday(&tv, nullptr);

    time_t      sec  = tv.tv_sec;
    int         ms   = tv.tv_usec / 1000;

    tm tmNow{};

    localtime_r(&sec, &tmNow);

    std::ostringstream oss;
    oss << std::dec << std::setfill('0')
        << std::setw(4) << (tmNow.tm_year + 1900) << '-'
        << std::setw(2) << (tmNow.tm_mon + 1)  << '-'
        << std::setw(2) << tmNow.tm_mday        << ' '
        << std::setw(2) << tmNow.tm_hour        << ':'
        << std::setw(2) << tmNow.tm_min         << ':'
        << std::setw(2) << tmNow.tm_sec         << '.'
        << std::setw(3) << ms;
    return oss.str();
}

uint64_t SystemAPI::getTimestamp(TimeUnit unit) {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();

    switch (unit) {
        case TimeUnit::SECOND:
            return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        case TimeUnit::MILLISECOND:
            return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        case TimeUnit::MICROSECOND:
            return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        case TimeUnit::NANOSECOND:
            return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        default:
            return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }
}

int SystemAPI::Read(int __fd, void* __buf, int __nbytes)
{
    errno = 0;
    int ret = read(__fd, __buf, __nbytes);
    if (ret == -1 && errno == EAGAIN)   //重试
        return 0;
    return ret;
}