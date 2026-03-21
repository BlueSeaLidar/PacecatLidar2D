#ifndef MANAGERLOG_H
#define MANAGERLOG_H

#include <vector>
#include <string>
#include <queue>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <cstring>

#ifdef _WIN32
#include <windows.h>      // 定义 HANDLE, CRITICAL_SECTION
#include <process.h>      // 定义 _beginthreadex
#else
#include <stdint.h>
#include <pthread.h>      // 定义 pthread_t, pthread_mutex_t, pthread_cond_t
#endif

#define MAXLIDAR 1	//暂只支持一个雷达

// 数据块结构，用于队列存储
struct LogDataBlock {
	int index;           // 雷达索引
	std::string data;    // 数据内容
	uint64_t timestamp;  // 时间戳

	LogDataBlock(int idx, const char* d, int size)
		: index(idx), data(d, size), timestamp(0) {}
};

class ManagerLog
{

#define LOGCACHE 1024*512
#define MAX_BINARY_LENGTH  (1024*1024*100)
#define MAX_FLODER_FILE_SIZE   5

public:
	static ManagerLog& getInstance();

	// 禁止所有拷贝和移动
private:
	ManagerLog(const ManagerLog&);
	ManagerLog& operator=(const ManagerLog&);

public:
	// 设置参数
	void setConfig(bool enabled, const std::string& root_path="", uint64_t min_disk_mb=100, uint64_t max_queue_bytes=4096);

	// 记录日志
	void setStreamLog(int index, char* data, int size);

	// 自动日志控制接口
	// 检测到丢包时调用，成功开启返回true，失败（磁盘满等）返回false
	// 如果正在稳定倒计时期间，会重置倒计时（时间后移）
	bool autoStartLog();
	// 数据正常时调用，内部处理3秒稳定期，满3秒后自动停止日志	
	void autoCheckStopLog();
	// 线程退出时调用，清理自动日志状态	
	void autoLogCleanup();

private:
	ManagerLog();
	~ManagerLog();

	bool init(std::string path, std::string streamName, int num);
	void uninit();
	bool saveLog(bool isSave);

	// 后台写入线程函数
#ifdef _WIN32
	static unsigned __stdcall writerThreadFunc(void* param);
#else
	static void* writerThreadFunc(void* param);
#endif
	// 实际写入磁盘的操作（从队列取数据）	
	void writeDataToDisk(const LogDataBlock& block);

	// 工具函数
	void checkFolderNameModify(int index);
	bool ReName(std::string oldpath, std::string newpath);
	bool MkDir(const std::string& strPath);
	void listFiles(std::string dir, std::vector<std::string>& folderlist);
	// 获取当前时间字符串
	std::string getCurrentTimeString();
	std::string getCurrentTime();
	std::string getCurrentTimeWithMs();
	// 获取当前可执行文件路径
	std::string getExecutablePath();
	// 检查目录是否可写
	bool IsDirWritable(const std::string& path);
	// 获取指定路径所在磁盘的剩余空间（字节）
	uint64_t getDiskFreeSpace(const std::string& path);
	// 检查磁盘空间是否充足
	bool checkDiskSpace(const std::string& path);

	// 平台无关的同步原语封装
	void initLock();
	void destroyLock();
	void lock();
	void unlock();
	void initQueueLock();
	void destroyQueueLock();
	void queueLock();
	void queueUnlock();
	void queueWait();       // 等待队列有空间（生产者用）
	void queueWaitData();   // 等待队列有数据（消费者用）
	void queueNotify();     // 通知队列有数据（唤醒消费者）
	void queueNotifySpace(); // 通知队列有空间（唤醒生产者）

private:
	// 启用开关
	bool m_enabled;

	std::vector<std::ofstream*> m_stream_loggers;//保存读取的所有数据流
	std::vector<uint64_t> m_stream_idx;//二进制流下标保存  每页下的下标
	std::vector<std::string> m_stream_fileName;//二进制流下标保存  每个文件的路径
	std::vector<std::vector<std::string> > m_stream_folderName;//二进制流下标保存  每个文件夹的路径
	std::vector<uint64_t> m_stream_page;//二进制流下标保存  页数
	std::vector<std::string> m_stream_time;//每个文件的保存时间
	std::string m_folder_name;//日志根目录

	bool m_isopen;

	std::vector<std::string> m_cacheBuffers;//缓存
	std::vector<int> m_cacheidxs;//缓存
	int m_cacheSize;

	std::string m_custom_log_path;//日志路径
	uint64_t m_minDiskSpaceMB;//磁盘最小容量

	// 自动日志控制状态
	bool m_logAutoStarted;
	bool m_waitingForStable;
	// C++98
#ifdef _WIN32
	uint64_t m_firstStableTime;  // Windows: GetTickCount64
#else
	timespec m_firstStableTime;  // Linux: clock_gettime
#endif

	// 线程和同步
#ifdef _WIN32
	HANDLE m_writerThread;
	HANDLE m_queueEvent;      // 队列有数据事件
	HANDLE m_spaceEvent;      // 队列有空间事件
	CRITICAL_SECTION m_mutex;
	CRITICAL_SECTION m_queueMutex;
#else
	pthread_t m_writerThread;
	pthread_mutex_t m_mutex;
	pthread_mutex_t m_queueMutex;
	pthread_cond_t m_queueCond;
	pthread_cond_t m_spaceCond;
#endif

	std::queue<LogDataBlock> m_dataQueue;
	uint64_t m_maxQueueBytes;		// 队列最大字节数限制（默认4096）
	uint64_t m_currentQueueBytes;	// 当前队列中数据的总字节数
	bool m_stopWriter;
	bool m_warnedFull;
};

#endif // MANAGERLOG_H
