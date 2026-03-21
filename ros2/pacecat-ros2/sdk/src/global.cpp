#include "../include/global.h"

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