#include "standard_interface.h"
#include "service/LidarWebService.h"
#include "managerlog.h"

PaceCatLidarSDK *PaceCatLidarSDK::m_sdk = new (std::nothrow) PaceCatLidarSDK();

PaceCatLidarSDK *PaceCatLidarSDK ::getInstance()
{
	return m_sdk;
}
void PaceCatLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}

PaceCatLidarSDK::PaceCatLidarSDK()
{
	m_idx = 0;
	m_checkservice = NULL;
}

PaceCatLidarSDK::~PaceCatLidarSDK()
{
}
RunConfig *PaceCatLidarSDK::getLidar(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
			return m_lidars.at(i);
	}

	return NULL;
}
int PaceCatLidarSDK::addLidarByPath(const char *cfg_file_name)
{
	RunConfig *cfg = new RunConfig;
	memset((char *)cfg, 0, sizeof(RunConfig));
	if (readConfig(cfg_file_name, cfg->runscript))
	{
		m_idx++;
		cfg->ID = m_idx;
		m_lidars.push_back(cfg);
		return m_idx;
	}
	else
		return 0;
}

bool PaceCatLidarSDK::delLidarByID(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			if (m_lidars.at(i)->state == WORK || m_lidars.at(i)->state == WORK_AND_WEB)
			{
				m_lidars.at(i)->state = STOP_ALL;
				sleep(1);
			}
			m_lidars.erase(m_lidars.begin() + i);
			return true;
		}
	}
	return false;
}

void PaceCatLidarSDK::setCallBackPtr(int ID, printfMsg ptr, void* context)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			m_lidars.at(i)->callback = ptr;
			m_lidars.at(i)->context = context;
		}
	}
}

bool PaceCatLidarSDK::openDev(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;
	if (strcmp(lidar->runscript.type, "uart") == 0 || strcmp(lidar->runscript.type, "vpc") == 0)
	{
		int fd = SystemAPI::open_serial_port(lidar->runscript.connectArg, lidar->runscript.connectArg2);
		if (fd <= 0)
		{
			return false;
		}
		lidar->fd = fd;

#ifdef __unix__
		if (pthread_create(&lidar->thread_data, NULL, lidar_thread_proc_uart, lidar) != 0)
			return false;
#elif _WIN32
		if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lidar_thread_proc_uart, lidar, 0, &lidar->thread_data) < 0)
			return false;

#endif
	}
	else if (strcmp(lidar->runscript.type, "udp") == 0)
	{
		int fd = SystemAPI::open_socket_port(lidar->runscript.local_port);
		if (fd <= 0)
		{
			return false;
		}
		lidar->fd = fd;
#ifdef __unix__
		if (pthread_create(&lidar->thread_data, NULL, lidar_thread_proc_udp, lidar) != 0)
			return false;
#elif _WIN32
		if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lidar_thread_proc_udp, lidar, 0, &lidar->thread_data) < 0)
			return false;
#endif
	}
	else
	{
		return false;
	}
	lidar->state = WORK;

	// 启动web本地服务，主要用于可视化查看雷达点云是否正常，SDK单独集成则不需要
	if (lidar->runscript.is_open_service)
	{
		lidar->state = WORK_AND_WEB;
		if (OpenLocalService(ID))
			printf("Please control it through a browser and enter the default address: http://localhost:%d\n", lidar->runscript.service_port);
		else
		{
			printf("Open web thread failed,port:%d\n", lidar->runscript.service_port);
			return false;
		}
	}

	// 启动雷达心跳包线程以及检测当前在线雷达数量
	OpenHeartService();

	// 判定数据线程是否正常运行
	int index = 100;
	while (lidar->action < ONLINE && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action >= ONLINE) {
		return true;
	}

	return false;
}

bool PaceCatLidarSDK::GetDevInfo(int ID, EEpromV101 *eepromv101)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	lidar->action = GETALLPARAMS;
	int index = 30;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(200);
		index--;
	}

	if (lidar->action == FINISH)
	{
		if (strcmp(lidar->recv_cmd, "OK") == 0)
		{
			memcpy(eepromv101, &lidar->eepromv101, sizeof(EEpromV101));
			return true;
		}
	}
	return false;
}
bool PaceCatLidarSDK::SetDevInfo(RunConfig *lidar, int num, char *cmd, int mode)
{
	lidar->mode = mode;
	lidar->send_len = num;
	memset(lidar->recv_cmd, 0, sizeof(lidar->recv_cmd));
	lidar->recv_len = 0;
	strcpy(lidar->send_cmd, cmd);
	lidar->action = SETPARAM;
	int index = 20;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action == FINISH)
	{
		if (strcmp(lidar->recv_cmd, "OK") == 0)
		{
			printf("%s OK\n", lidar->send_cmd);
			return true;
		}
		else if (strcmp(lidar->recv_cmd, "NG") == 0)
		{
			printf("%s NG\n", lidar->send_cmd);
			return false;
		}
	}
	return false;
}

// 释放连接
void PaceCatLidarSDK::StopDev(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			m_lidars.at(i)->action = OFFLINE;
		}
	}
}

const char *PaceCatLidarSDK::getVersion()
{
	return SDKVERSION;
}

void *lidar_service(void *param)
{
	int *ID = (int *)param;
	PaceCatLidarSDK::PaceCatLidarSDK::getInstance()->getLidar(*ID)->webservice->run(*ID);
	return SUCCESS;
}

// 打开本地服务
bool PaceCatLidarSDK::OpenLocalService(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;
	lidar->webservice = new LidarWebService(lidar->runscript.service_port);
#ifdef __unix__
	if (pthread_create(&lidar->thread_web, NULL, lidar_service, &lidar->ID) != 0)
		return false;
#elif _WIN32
	if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lidar_service, &lidar->ID, 0, &lidar->thread_web) < 0)
		return false;
#endif
	return true;
}
// 关闭本地服务
bool PaceCatLidarSDK::CloseLocalService(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	if (m_lidars.at(ID)->state == WORK_AND_WEB)
	{
		m_lidars.at(ID)->state = WORK;
		return true;
	}

	return false;
}

bool PaceCatLidarSDK::OpenHeartService()
{
	if (m_checkservice == NULL)
	{
		m_checkservice = new LidarCheckService();
	}
	m_checkservice->run();
	return true;
}

bool PaceCatLidarSDK::CloseHeartService()
{
	m_checkservice->stop();
	return true;
}

// 启停雷达测距
bool PaceCatLidarSDK::ControlDrv(int ID, int num, char *cmd)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	lidar->mode = C_PACK;
	lidar->send_len = num;
	strcpy(lidar->send_cmd, cmd);
	lidar->action = CONTROL;

	int index = 30;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action == FINISH)
	{
		printf("%s OK\n", lidar->send_cmd);
		return true;
	}
	else if (index == 0 && strcmp(lidar->send_cmd, "LRESTH") == 0)
	{
		printf("%s OK\n", lidar->send_cmd);
		return true;
	}

	return false;
}

bool PaceCatLidarSDK::ZoneSection(int ID, char section)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	// 判断传入的防区是否合法
	if ((section >= 48 && section <= 57) || (section >= 65 && section <= 70))
	{
		char tmp[12] = {0};
		sprintf(tmp, "LSAZN:%cH", section);
		return SetDevInfo(lidar, strlen(tmp), tmp, S_PACK);
	}

	return false;
}

bool PaceCatLidarSDK::SetUDP(int ID, char *ip, char *mask, char *gateway, int port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char result[64] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(1, ip, mask, gateway, port, result))
	{
		return false;
	}
	char tmp[128] = {0};
	sprintf(tmp, "LSUDP:%sH", result);
	lidar->mode = S_PACK;
	lidar->send_len = strlen(tmp);
	strcpy(lidar->send_cmd, tmp);
	lidar->action = SETPARAM;
	// 修改ip后没有返回值
	return true;
}

bool PaceCatLidarSDK::SetDST(int ID, char *ip, int port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char result[50] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(0, ip, (char *)"", (char *)"", port, result))
	{
		return false;
	}
	char tmp[64] = {0};
	sprintf(tmp, "LSDST:%sH", result);
	return SetDevInfo(lidar, strlen(tmp), tmp, S_PACK);
}

bool PaceCatLidarSDK::SetRPM(int ID, int RPM)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	// 对传入的格式校验
	if (RPM < 300 || RPM > 3000)
	{
		return false;
	}
	char cmd[16] = {0};
	sprintf(cmd, "LSRPM:%dH", RPM);
	return SetDevInfo(lidar, strlen(cmd), cmd, C_PACK);
}

bool PaceCatLidarSDK::SetTFX(int ID, bool tfx)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSTFX:%dH", tfx);
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool PaceCatLidarSDK::SetDSW(int ID, bool dsw)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LFFF%dH", dsw);
	return SetDevInfo(lidar, strlen(cmd), cmd, C_PACK);
}

bool PaceCatLidarSDK::SetSMT(int ID, bool smt)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSSS%dH", smt);
	return SetDevInfo(lidar, strlen(cmd), cmd, C_PACK);
}

bool PaceCatLidarSDK::SetPST(int ID, int mode)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	if (mode <= 0 || mode > 3)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSPST:%dH", mode);
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool PaceCatLidarSDK::SetDID(int ID, unsigned int did)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSDID:%dH", did);
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool PaceCatLidarSDK::SetNTP(int ID, char *ntp_ip, unsigned int port, bool enable)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[64];
	char ip_1[4];
	char ip_2[4];
	char ip_3[4];
	char ip_4[4];
	ip_1[3] = '\0';
	ip_2[3] = '\0';
	ip_3[3] = '\0';
	ip_4[3] = '\0';

	int idx[3];
	int index = 0;
	int ip_len = strlen(ntp_ip);
	for (int i = 0; i < ip_len; i++)
	{
		if (ntp_ip[i] == '.')
		{
			idx[index] = i;
			index++;
		}
	}
	if (index != 3)
	{
		printf("ntp ip set error!\n");
		return false;
	}
	else
	{
		memcpy(ip_1, &ntp_ip[0], idx[0]);
		memcpy(ip_2, &ntp_ip[idx[0] + 1], idx[1] - idx[0] - 1);
		memcpy(ip_3, &ntp_ip[idx[1] + 1], idx[2] - idx[1] - 1);
		memcpy(ip_4, &ntp_ip[idx[2] + 1], ip_len - idx[2]);
		sprintf(cmd, "LSNTP:%d,%03d.%03d.%03d.%03d,%05dH", enable, atoi(ip_1), atoi(ip_2), atoi(ip_3), atoi(ip_4), port);
		return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
	}
	return false;
}

bool PaceCatLidarSDK::SetPTP(int ID, bool enable)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = { 0 };
	sprintf(cmd, "LSPTP:%dH", enable);
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

bool PaceCatLidarSDK::SetRES(int ID, unsigned int res)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSRES:%dH", res);
	return SetDevInfo(lidar, strlen(cmd), cmd, S_PACK);
}

std::vector<DevConnInfo> PaceCatLidarSDK::getLidarsList()
{
	if (!OpenHeartService()) 
	{
		std::vector<DevConnInfo> tmp;
		return tmp;
	}
	return m_checkservice->getLidarsList();
}

int PaceCatLidarSDK::QueryIDByIp(char* ip)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		char* lidar_type = m_lidars[i]->runscript.type;
		int lidar_state = m_lidars.at(i)->state;
		char* lidar_ip = m_lidars[i]->runscript.connectArg;

		//只探测当前运行的雷达
		if (strcmp(lidar_type, "udp") == 0)
		{
			if (lidar_state == WORK || lidar_state == WORK_AND_WEB)
			{
				if (strcmp(lidar_ip, ip) == 0)
				{
					return m_lidars.at(i)->ID;
				}
			}
		}
	}
	return -1;
}

void PaceCatLidarSDK::SetRecordConfig(bool enabled, const std::string& root_path, uint64_t min_disk_mb, uint64_t max_queue_bytes)
{
	// 设置录制参数
	ManagerLog::getInstance().setConfig(enabled, root_path, min_disk_mb, max_queue_bytes);
}