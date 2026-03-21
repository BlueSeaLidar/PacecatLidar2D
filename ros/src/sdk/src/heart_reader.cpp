#include "../include/reader.h"
#include "../include/global.h"

#define HEART_INTERVAL 2 //心跳包检查间隔

typedef struct
{
	std::string sn;
	std::string ip;
	int flag;				//during a time and not recv heart package
	bool isonline;
}ConnectInfo;

typedef struct
{
	std::vector<ConnectInfo> lidars;
	bool isrun;
	std::string adapter;
	std::vector<std::string> target_ips;
	pthread_t thr;
}HeartInfo;

void LogHeartEvent(const std::string& sn, const std::string& type, const std::string& ip, const std::string& event)
{
    std::string result = "lidar " + event + ": sn=" + sn + " type=" + type + " ip=" + ip + " dt=" + SystemAPI::GetTimeYMDHMSms();
    DEBUG("%s", result.c_str());
}

void LogHeartMsg(char *buf_data, int buf_len)
{
    if (buf_len == sizeof(DevInfoV101))
    {
        DevInfoV101 *dvi = (DevInfoV101 *)buf_data;

        DEBUG("timestamp:%u.%u sn:%s model:%s id:%d rpm:%.1f", dvi->timestamp[0], dvi->timestamp[1], dvi->dev_sn, dvi->dev_type, dvi->dev_id, dvi->rpm / 10.0);
        char tmp_IPv4[16] = {0};
        char tmp_mask[16] = {0};
        char tmp_gateway[16] = {0};
        char tmp_srv_ip[16] = {0};

        sprintf(tmp_IPv4, "%d.%d.%d.%d", dvi->ip[0], dvi->ip[1], dvi->ip[2], dvi->ip[3]);
        sprintf(tmp_mask, "%d.%d.%d.%d", dvi->mask[0], dvi->mask[1], dvi->mask[2], dvi->mask[3]);
        sprintf(tmp_gateway, "%d.%d.%d.%d", dvi->gateway[0], dvi->gateway[1], dvi->gateway[2], dvi->gateway[3]);
        sprintf(tmp_srv_ip, "%d.%d.%d.%d", dvi->remote_ip[0], dvi->remote_ip[1], dvi->remote_ip[2], dvi->remote_ip[3]);
        DEBUG("lidar_ip:%s lidar_mask:%s lidar_gateway:%s lidar_port:%d upload_ip:%s upload_port:%d", tmp_IPv4, tmp_mask, tmp_gateway, dvi->port, tmp_srv_ip, dvi->remote_udp);
        
		std::string alarmMsg;
        if (dvi->alarm[FUN_LV_1])
            alarmMsg += " Observation Zone,";
        if (dvi->alarm[FUN_LV_2])
            alarmMsg += " Alert Zone,";
        if (dvi->alarm[FUN_LV_3])
            alarmMsg += " Alarm Zone,";
        if (dvi->alarm[FUN_COVER])
            alarmMsg += " Masking,";
        if (dvi->alarm[FUN_NO_DATA])
            alarmMsg += " No data,";
        if (!dvi->alarm[FUN_ZONE_ACTIVE])
            alarmMsg += " No zone active,";
        if (dvi->alarm[FUN_SYS_ERR])
            alarmMsg += " System error,";
        int v = dvi->status & 0x7;
        if (v == 1 || v == 3)
        {
            if (!dvi->alarm[FUN_NET_LINK])
                alarmMsg += " NET disconnected,";
        }
        else if (v == 2)
        {
            if (!dvi->alarm[FUN_USB_LINK])
                alarmMsg += " USB disconnected,";
        }
        if (dvi->alarm[FUN_UPDATING])
            alarmMsg += " System Upgrade,";
        if (dvi->alarm[FUN_ZONE_DEFINED])
            alarmMsg += " Zone not defined,";

        if(!alarmMsg.empty())
            DEBUG("lidar_alarm:%s",alarmMsg.c_str());
    }
}

void *HeartThreadProc(void *p);

HReader StartHeartReader(std::string adapter, std::vector<std::string> targetips)
{
	HeartInfo *info = new HeartInfo;
	info->isrun = true;
	info->adapter = adapter;
	info->target_ips = targetips;
	pthread_create(&info->thr, NULL, HeartThreadProc, info);
	return info;	
}

void StopHeartReader(HReader hr)
{
	HeartInfo *info = (HeartInfo*)hr;
    info->isrun = false;
    pthread_join(info->thr, NULL);
	delete info;
}

void* HeartThreadProc(void* p)
{
	HeartInfo *heartinfo = (HeartInfo *)p;
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes)) < 0)
	{
		DEBUG("socket init error:%d", SystemAPI::getLastError());
		SystemAPI::closefd(sock);
		return NULL;
	}
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(HeartPort);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (iResult != 0)
	{
		DEBUG("bind port failed:%d", SystemAPI::getLastError());
		SystemAPI::closefd(sock);
		return NULL;  
	}
	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = SystemAPI::get_interface_ip(heartinfo->adapter.c_str());
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
	{
		DEBUG("add broadcast error:%d", SystemAPI::getLastError());
		SystemAPI::closefd(sock);
		return NULL;  
	}

	uint64_t currentTimeStamp = SystemAPI::GetTimeStampNow();
	uint64_t tto = currentTimeStamp + HEART_INTERVAL * 1000;
	socklen_t sz = sizeof(addr);
	while (heartinfo->isrun)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(sock, &fds);
		struct timeval to = {0, 100};
		int ret = select(sock + 1, &fds, NULL, NULL, &to);
		if (ret > 0)
		{
			char raw[4096];
			int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr *)&addr, &sz);
			if (dw >= (int)sizeof(DevInfoV101) && raw[0]=='L' && raw[1]=='i' && raw[2]=='D' && raw[3]=='A')
			{
				DevInfoV101 *devheart = (DevInfoV101 *)raw;
				std::string sn = BaseAPI::stringfilter(devheart->dev_sn, 20);
				char tmp_ip[16] = {0};
				sprintf(tmp_ip, "%d.%d.%d.%d", devheart->ip[0], devheart->ip[1], devheart->ip[2], devheart->ip[3]);
				std::string ip = BaseAPI::stringfilter(tmp_ip, 16);
	
				for (unsigned int i = 0; i < heartinfo->lidars.size(); i++)
				{
					if (sn == heartinfo->lidars[i].sn)
					{
						heartinfo->lidars[i].flag = true;
						if (!heartinfo->lidars[i].isonline)
						{
							heartinfo->lidars[i].isonline = true;
							LogHeartEvent(sn, BaseAPI::stringfilter(devheart->dev_type, 16), ip, "online");
						}
						// 打印具体信息
						LogHeartMsg(raw, dw);	
					}
				}

				bool should_add = false;

				for (const auto& target : heartinfo->target_ips) 
				{
					if (ip == target) 
					{
						should_add = true;
						break;
					}
				}

				if (should_add) 
				{
					for (const auto& lidar : heartinfo->lidars) 
					{
						if (lidar.ip == ip) 
						{
							should_add = false;
							break;
						}
					}
				}

				if (should_add)
				{
					ConnectInfo info;
					info.ip = ip;
					info.sn = sn;
					info.isonline = true;
					info.flag = true;
					heartinfo->lidars.push_back(info);
					LogHeartEvent(sn, BaseAPI::stringfilter(devheart->dev_type, 16), ip, "online");
				}
			}
		}
		// check is outtime
		currentTimeStamp = SystemAPI::GetTimeStampNow();
		if (currentTimeStamp > tto)
		{
			for (unsigned int i = 0; i < heartinfo->lidars.size(); i++)
			{
				if (heartinfo->lidars[i].isonline == true && !heartinfo->lidars[i].flag)
				{
					heartinfo->lidars[i].isonline = false;
					LogHeartEvent(heartinfo->lidars[i].sn, "", heartinfo->lidars[i].ip, "offline");
				}
				heartinfo->lidars[i].flag = false;
			}
			tto = currentTimeStamp + HEART_INTERVAL * 1000;
		}
	}
	heartinfo->lidars.clear();
	heartinfo->isrun = false;
	SystemAPI::closefd(sock);
	return NULL;
}