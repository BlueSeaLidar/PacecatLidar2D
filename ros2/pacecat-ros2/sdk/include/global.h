#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <cerrno>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>

enum TimeUnit {
    SECOND,
    MILLISECOND,
    MICROSECOND,
    NANOSECOND
};

namespace BaseAPI
{
    std::string stringfilter(char *str,int num);
}

namespace SystemAPI
{
    int getLastError();    
    int closefd(int __fd);
    in_addr_t get_interface_ip(const char *ifname);
    uint64_t GetTimeStampNow();
    std::string GetTimeYMDHMSms();
    uint64_t getTimestamp(TimeUnit unit);
    //uart
    int Read(int __fd, void* __buf, int __nbytes);
}

extern int g_uart_record_enabled;

#endif