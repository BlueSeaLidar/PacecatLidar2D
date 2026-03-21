#pragma once

#include"Global.h"
#include "LidarDataProcess.h"

class LidarCheckService
{
public:
	LidarCheckService();
	~LidarCheckService();

	void run();
	void stop();
	void clear();
	std::vector<DevConnInfo> getLidarsList();
	static void getTime_HMS(char*data);
	void uartDevInfo();
	void uptodate(DevConnInfo data);
	void callbackHeart(DevConnInfo* info);
	
public:
	bool m_close_service;
	std::vector<DevConnInfo> m_infos;

private:
	unsigned long m_thread_heart;
};
void* thread_heart(void* p);

//static void getTime_HMS(char*data);


