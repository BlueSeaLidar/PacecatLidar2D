#include "managerlog.h"
#include <sstream>

#ifdef _WIN32
#include <io.h>           // _access, _mkdir
#include <direct.h>       // _mkdir
#else
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>
#include <dirent.h>
#include <sys/statvfs.h>
#endif

// 宏定义
#ifdef _WIN32
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#define RMDIR(a) _rmdir((a))
#define PATH_SEPARATOR "\\"
#define GETPID() _getpid()
#else
#define ACCESS access
#define MKDIR(a) mkdir((a),0755)
#define RMDIR(a) rmdir((a))
#define PATH_SEPARATOR "/"
#define GETPID() getpid()
#endif

static inline std::string intToString(int value) {
	std::ostringstream oss;
	oss << value;
	return oss.str();
}

// 单例实现
ManagerLog& ManagerLog::getInstance() {
	static ManagerLog instance;
	return instance;
}

// 平台无关的锁实现
#ifdef _WIN32
void ManagerLog::initLock() {
	InitializeCriticalSection(&m_mutex);
}
void ManagerLog::destroyLock() {
	DeleteCriticalSection(&m_mutex);
}
void ManagerLog::lock() {
	EnterCriticalSection(&m_mutex);
}
void ManagerLog::unlock() {
	LeaveCriticalSection(&m_mutex);
}
void ManagerLog::initQueueLock() {
	InitializeCriticalSection(&m_queueMutex);
	m_queueEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	m_spaceEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
}
void ManagerLog::destroyQueueLock() {
	DeleteCriticalSection(&m_queueMutex);
	CloseHandle(m_queueEvent);
	CloseHandle(m_spaceEvent);
}
void ManagerLog::queueLock() {
	EnterCriticalSection(&m_queueMutex);
}
void ManagerLog::queueUnlock() {
	LeaveCriticalSection(&m_queueMutex);
}
void ManagerLog::queueWait() {
	queueUnlock();
	WaitForSingleObject(m_spaceEvent, INFINITE);
	queueLock();
}
void ManagerLog::queueWaitData() {
	queueUnlock();
	WaitForSingleObject(m_queueEvent, INFINITE);
	queueLock();
}
void ManagerLog::queueNotify() {
	SetEvent(m_queueEvent);
}
void ManagerLog::queueNotifySpace() {
	SetEvent(m_spaceEvent);
}
#else
void ManagerLog::initLock() {
	pthread_mutex_init(&m_mutex, NULL);
}
void ManagerLog::destroyLock() {
	pthread_mutex_destroy(&m_mutex);
}
void ManagerLog::lock() {
	pthread_mutex_lock(&m_mutex);
}
void ManagerLog::unlock() {
	pthread_mutex_unlock(&m_mutex);
}
void ManagerLog::initQueueLock() {
	pthread_mutex_init(&m_queueMutex, NULL);
	pthread_cond_init(&m_queueCond, NULL);
	pthread_cond_init(&m_spaceCond, NULL);
}
void ManagerLog::destroyQueueLock() {
	pthread_mutex_destroy(&m_queueMutex);
	pthread_cond_destroy(&m_queueCond);
	pthread_cond_destroy(&m_spaceCond);
}
void ManagerLog::queueLock() {
	pthread_mutex_lock(&m_queueMutex);
}
void ManagerLog::queueUnlock() {
	pthread_mutex_unlock(&m_queueMutex);
}
void ManagerLog::queueWait() {
	pthread_cond_wait(&m_spaceCond, &m_queueMutex);
}
void ManagerLog::queueWaitData() {
	pthread_cond_wait(&m_queueCond, &m_queueMutex);
}
void ManagerLog::queueNotify() {
	pthread_cond_signal(&m_queueCond);
}
void ManagerLog::queueNotifySpace() {
	pthread_cond_signal(&m_spaceCond);
}
#endif

void ManagerLog::setConfig(bool enabled, const std::string& root_path, uint64_t min_disk_mb, uint64_t max_queue_bytes) {
	lock();
	m_enabled = enabled;
	m_custom_log_path = root_path;
	m_minDiskSpaceMB = min_disk_mb;
	m_maxQueueBytes = max_queue_bytes;
	unlock();
}

void ManagerLog::setStreamLog(int index, char* data, int size)
{
	// 如果未启用，直接返回
	if (!m_enabled) {
		return;
	}

	lock();

	// 日志未打开状态
	if (!m_isopen)
	{
		// 缓存大小为0
		if (m_cacheSize == 0 || size == 0) {
			unlock();
			return;
		}
		else
		{
			std::string& buf = m_cacheBuffers.at(index);
			int& idx = m_cacheidxs.at(index);
			int bufSize = (int)buf.size();

			if (bufSize < m_cacheSize) {
				// 未满
				int remaining = m_cacheSize - bufSize;
				int needFill = size < remaining ? size : remaining;
				buf.append(data, needFill);

				if (size > needFill) {
					// 还有剩余，环绕写入开头
					int remain = size - needFill;
					for (int i = 0; i < remain; i++) {
						buf[i] = data[needFill + i];
					}
					idx = remain;
				}
				else {
					idx = (int)buf.size();
				}
			}
			else {
				// 已满，环形覆盖
				for (int i = 0; i < size; i++) {
					buf[(idx + i) % m_cacheSize] = data[i];
				}
				idx = (idx + size) % m_cacheSize;
			}
		}
		unlock();
		return;
	}
	else
	{
		// 检查磁盘空间，不足则停止写入
		if (!checkDiskSpace(m_folder_name)) {
			// 磁盘满时自动停止日志（如果是自动开启的）
			if (m_logAutoStarted) {
				printf("Disk full, auto stopping log\n");
				saveLog(false);
				m_logAutoStarted = false;
				m_waitingForStable = false;
			}
			unlock();
			return;
		}
		unlock();

		// 将数据放入队列
		queueLock();

		//printf("1111:%zu, bytes:%llu\n", m_dataQueue.size(), (unsigned long long)m_currentQueueBytes);

		// 等待队列有空间
		while (m_currentQueueBytes + size > m_maxQueueBytes && !m_stopWriter && !m_dataQueue.empty()) {
			if (!m_warnedFull) {
				printf("Log queue full (%llu bytes), blocking...\n", (unsigned long long)m_currentQueueBytes);
				m_warnedFull = true;
			}
			queueWait();
		}

		if (m_stopWriter) {
			queueUnlock();
			return;
		}

		// 恢复检测：从满恢复到非满
		if (m_warnedFull && m_currentQueueBytes + size <= m_maxQueueBytes) {
			//printf("Log queue recovered (bytes: %llu)\n", (unsigned long long)m_currentQueueBytes);
			m_warnedFull = false;
		}

		m_dataQueue.push(LogDataBlock(index, data, size));
		m_currentQueueBytes += size;  // 累加字节数

		queueUnlock();

		// 通知写入线程
		queueNotify();
	}
}

// 后台写入线程函数
#ifdef _WIN32
unsigned __stdcall ManagerLog::writerThreadFunc(void* param)
#else
void* ManagerLog::writerThreadFunc(void* param)
#endif
{
	ManagerLog* self = (ManagerLog*)param;
	//printf("start writer thread\n");
	while (!self->m_stopWriter) {
		LogDataBlock block(0, NULL, 0);
		bool hasData = false;

		self->queueLock();

		// 等待队列有数据
		while (self->m_dataQueue.empty() && !self->m_stopWriter) {
			self->queueWaitData();
		}

		if (self->m_stopWriter) {
			self->queueUnlock();
			break;
		}

		if (self->m_dataQueue.empty()) {
			self->queueUnlock();
			continue;
		}

		block = self->m_dataQueue.front();
		self->m_dataQueue.pop();

		// 减去出队数据的字节数
		self->m_currentQueueBytes -= block.data.size();

		//printf("2222:%zu, bytes:%llu\n", self->m_dataQueue.size(), (unsigned long long)self->m_currentQueueBytes);

		hasData = true;

		// 通知生产者队列有空间了
		self->queueNotifySpace();

		self->queueUnlock();

		if (hasData) {
			self->writeDataToDisk(block);
		}		
	}
	//printf("stop writer thread\n");
#ifdef _WIN32
	return 0;
#else
	return NULL;
#endif
}

//#include <thread>
//#include <chrono>

// 实际写入磁盘
void ManagerLog::writeDataToDisk(const LogDataBlock& block)
{
	//std::this_thread::sleep_for(std::chrono::microseconds(300));
	//usleep(300);

	int index = block.index;
	const char* data = block.data.data();
	int size = (int)block.data.size();

	if (m_stream_loggers.size() < (size_t)(index + 1)) {
		return;
	}

	// 先写入缓存数据（如果有）
	int tmpidx = m_cacheidxs[index];
	int tmpsize = (int)m_cacheBuffers[index].size();
	std::string tmpdata = m_cacheBuffers[index];
	if (m_cacheBuffers[index].size())
	{
		m_stream_loggers.at(index)->write(tmpdata.data() + tmpidx, tmpsize - tmpidx);
		m_stream_loggers.at(index)->write(tmpdata.data(), tmpidx);

		m_cacheBuffers[index].clear();
		m_cacheidxs[index] = 0;
		m_stream_idx[index] = tmpsize;
	}

	// 写入实际数据
	int idx = (int)m_stream_idx.at(index);
	if (idx + size < MAX_BINARY_LENGTH)
	{
		m_stream_loggers.at(index)->write(data, size);
		m_stream_loggers.at(index)->flush();
	}
	else if (idx + size == MAX_BINARY_LENGTH)
	{
		m_stream_loggers.at(index)->write(data, size);
		m_stream_loggers.at(index)->flush();
		m_stream_loggers.at(index)->close();
		m_stream_page[index]++;
		if (m_stream_page[index] % MAX_FLODER_FILE_SIZE == 0)
		{
			std::string oldfoldname = m_stream_folderName.at(index).at(m_stream_folderName.at(index).size() - 1);
			std::string oldfoldname_add = oldfoldname + '-' + getCurrentTime();
			m_stream_folderName[index][m_stream_folderName.at(index).size() - 1] = oldfoldname_add;
			ReName(oldfoldname, oldfoldname_add);
			std::string newfoldname = m_folder_name + getCurrentTime();
			MkDir(newfoldname);
			m_stream_folderName[index].push_back(newfoldname);
		}
		std::string filename3 = m_stream_folderName.at(index).at(m_stream_folderName.at(index).size() - 1) + PATH_SEPARATOR + getCurrentTime() + "_" + intToString(index + 1) + ".dump";

		std::ofstream* logger = new std::ofstream;
		logger->open(filename3.c_str(), std::ios_base::app | std::ios_base::binary);
		m_stream_loggers[index] = logger;

		checkFolderNameModify(index);
	}
	else if (idx + size > MAX_BINARY_LENGTH)
	{
		int sum = size;//待写入字节数
		int consume = MAX_BINARY_LENGTH - idx;//本次写入字节数
		int num = (idx + size) / MAX_BINARY_LENGTH;//需要重复写入次数

		for (int i = 0; i < num; i++)
		{
			m_stream_loggers.at(index)->write(data, consume);
			m_stream_loggers.at(index)->flush();
			m_stream_loggers.at(index)->close();
			sum = size - consume;
			if (sum > MAX_BINARY_LENGTH)
				consume = MAX_BINARY_LENGTH;
			else if (sum > 0 && sum <= MAX_BINARY_LENGTH)
				consume = sum;
			m_stream_page[index]++;
			if (m_stream_page[index] % MAX_FLODER_FILE_SIZE == 0)
			{
				std::string oldfoldname = m_stream_folderName.at(index).at(m_stream_folderName.at(index).size() - 1);
				std::string oldfoldname_add = oldfoldname + '-' + getCurrentTime();
				m_stream_folderName[index][m_stream_folderName.at(index).size() - 1] = oldfoldname_add;
				ReName(oldfoldname, oldfoldname_add);

				std::string newfoldname = m_folder_name + getCurrentTime();
				MkDir(newfoldname);
				m_stream_folderName[index].push_back(newfoldname);
			}

			std::string filename3 = m_stream_folderName.at(index).at(m_stream_folderName.at(index).size() - 1) + PATH_SEPARATOR + getCurrentTime() + "_" + intToString(index + 1) + ".dump";
			std::ofstream* logger = new std::ofstream;
			logger->open(filename3.c_str(), std::ios_base::app | std::ios_base::binary);
			m_stream_loggers[index] = logger;
			checkFolderNameModify(index);
			if (sum == 0)
				break;
		}
	}
	m_stream_idx[index] = (idx + size) % MAX_BINARY_LENGTH;
}

ManagerLog::ManagerLog()
{
	m_enabled = false;	//默认不启用
	m_isopen = false;
	m_cacheSize = LOGCACHE;
	m_minDiskSpaceMB = 100;
	m_maxQueueBytes = 4096;
	m_currentQueueBytes = 0;
	m_logAutoStarted = false;
	m_waitingForStable = false;
	m_stopWriter = false;
	m_warnedFull = false;
	m_cacheBuffers.resize(MAXLIDAR);
	m_cacheidxs.resize(MAXLIDAR);
	initLock();
	initQueueLock();
#ifdef _WIN32
	m_writerThread = NULL;
	m_firstStableTime = 0;
#else
	m_firstStableTime.tv_sec = 0;
	m_firstStableTime.tv_nsec = 0;
#endif
}

ManagerLog::~ManagerLog()
{
	// 停止写入线程
	m_stopWriter = true;
	queueNotify();
	queueNotifySpace();
#ifdef _WIN32
	if (m_writerThread != NULL) {
		WaitForSingleObject(m_writerThread, INFINITE);
		CloseHandle(m_writerThread);
	}
#else
	if (m_writerThread) {
		pthread_join(m_writerThread, NULL);
	}
#endif

	// 清空队列
	queueLock();
	while (!m_dataQueue.empty()) {
		m_dataQueue.pop();
	}
	m_currentQueueBytes = 0;
	queueUnlock();

	for (size_t i = 0; i < m_stream_loggers.size(); i++)
	{
		if (m_stream_loggers.at(i) != NULL)
		{
			m_stream_loggers.at(i)->close();
			delete m_stream_loggers[i];
		}
	}

	destroyLock();
	destroyQueueLock();
}

void ManagerLog::checkFolderNameModify(int index)
{
	std::vector<std::string>folderlist;
	listFiles(m_folder_name, folderlist);

	std::vector<std::string>paths = m_stream_folderName[index];
	for (size_t i = 0; i < folderlist.size(); i++)
	{
		if (folderlist[i].find('-') == std::string::npos)
		{
			//说明之前重命名的时候文件还在退出状态，未关闭所以继续重命名	
			for (size_t j = 0; j < paths.size(); j++)
			{
				if (paths[j].find('-') == std::string::npos)
					continue;

				std::string tmppath = m_folder_name + folderlist[i];
				printf("Recover folder:%s	%s\n", tmppath.c_str(), paths[j].c_str());
				if (paths[j].find(tmppath) != std::string::npos)
				{
					ReName(tmppath, paths[j]);
					break;
				}
			}
		}
	}
}

bool ManagerLog::init(std::string path, std::string allName, int num)
{
	if (m_isopen == true)
	{
		return true;
	}

	m_folder_name = path;
	MkDir(path);
	m_stream_idx.resize(num);
	m_stream_page.resize(num, 0);
	m_cacheBuffers.resize(num);

	std::string filename, filename2, filename3;
	for (int i = 0; i < num; i++)
	{
		if (!allName.empty())
		{
			std::string foldname = path + getCurrentTime();
			MkDir(foldname);
			std::string filename3 = foldname + PATH_SEPARATOR + getCurrentTime() + "_" + intToString(i + 1) + ".dump";
			std::ofstream* logger = new std::ofstream;
			logger->open(filename3.c_str(), std::ios_base::app | std::ios_base::binary);
			m_stream_loggers.push_back(logger);
			m_stream_fileName.push_back(filename3);
			std::vector<std::string>tmpfolderName;
			tmpfolderName.push_back(foldname);
			m_stream_folderName.push_back(tmpfolderName);
		}
	}

	m_isopen = true;

	// 启动后台写入线程
	m_stopWriter = false;
#ifdef _WIN32
	m_writerThread = (HANDLE)_beginthreadex(NULL, 0, writerThreadFunc, this, 0, NULL);
#else
	pthread_create(&m_writerThread, NULL, writerThreadFunc, this);
#endif

	return true;
}

void ManagerLog::uninit()
{
	m_isopen = false;
	m_stream_fileName.clear();

	// 停止写入线程
	m_stopWriter = true;
	queueNotify();
	queueNotifySpace();
#ifdef _WIN32
	if (m_writerThread != NULL) {
		WaitForSingleObject(m_writerThread, INFINITE);
		CloseHandle(m_writerThread);
		m_writerThread = NULL;
	}
#else
	if (m_writerThread) {
		pthread_join(m_writerThread, NULL);
		m_writerThread = 0;
	}
#endif

	// 清空队列
	queueLock();
	while (!m_dataQueue.empty()) {
		m_dataQueue.pop();
	}
	m_currentQueueBytes = 0;
	queueUnlock();

	for (size_t i = 0; i < m_stream_loggers.size(); i++)
	{
		m_stream_loggers.at(i)->close();
		delete m_stream_loggers[i];
		std::string oldfoldname = m_stream_folderName.at(i).at(m_stream_folderName.at(i).size() - 1);
		std::string newfoldname = oldfoldname + '-' + getCurrentTime();
		ReName(oldfoldname, newfoldname);
	}

	m_cacheBuffers.clear();
	m_cacheidxs.clear();
	m_cacheBuffers.resize(MAXLIDAR);
	m_cacheidxs.resize(MAXLIDAR);

	m_stream_folderName.clear();
	m_stream_page.clear();
	m_stream_time.clear();
	m_stream_loggers.clear();
}

bool ManagerLog::ReName(std::string oldpath, std::string newpath)
{
	int ret = rename(oldpath.c_str(), newpath.c_str());
	if (ret == 0)
		return true;
	else
		return false;
}

bool ManagerLog::MkDir(const std::string& strPath)
{
	if (strPath.empty())
		return false;

	size_t nDirLen = strPath.length();
	char* pDirTemp = new char[nDirLen + 4];
	strPath.copy(pDirTemp, nDirLen + 1, 0);
	pDirTemp[nDirLen] = '\0';

	// 统一路径分隔符为 '/'
	for (size_t j = 0; j < nDirLen; j++) {
		if (pDirTemp[j] == '\\')
			pDirTemp[j] = '/';
	}

	// 末尾加 '/'
	if (pDirTemp[nDirLen - 1] != '/') {
		pDirTemp[nDirLen] = '/';
		pDirTemp[nDirLen + 1] = '\0';
		nDirLen++;
	}

	// 计算起始位置（平台相关）
	size_t start = 0;

#ifdef _WIN32
	// Windows: C:/ 路径跳过 "C:"
	if (nDirLen >= 2 && pDirTemp[1] == ':') {
		start = 2;
	}
	// Windows: UNC //server/share/ 找到第三个 /
	else if (nDirLen >= 2 && pDirTemp[0] == '/' && pDirTemp[1] == '/') {
		size_t slashCount = 0;
		for (size_t k = 2; k < nDirLen; k++) {
			if (pDirTemp[k] == '/') {
				slashCount++;
				if (slashCount == 2) {
					start = k;
					break;
				}
			}
		}
		if (slashCount < 2)
			start = 2;
	}
#else
	// Linux: 绝对路径跳过开头的 /
	if (pDirTemp[0] == '/') {
		start = 1;
	}
#endif

	// 公共循环逻辑（使用宏定义的 ACCESS 和 MKDIR）
	for (size_t i = start; i < nDirLen; i++) {
		if (pDirTemp[i] == '/') {
			pDirTemp[i] = '\0';

			// 跳过空路径
			if (strlen(pDirTemp) > 0) {
				int statu = ACCESS(pDirTemp, 0);
				if (statu != 0) {
					statu = MKDIR(pDirTemp);
					if (statu != 0) {
						delete[] pDirTemp;
						return false;
					}
				}
			}
			pDirTemp[i] = '/';
		}
	}

	delete[] pDirTemp;
	return true;
}

#ifdef __unix__
void ManagerLog::listFiles(std::string dir, std::vector<std::string>& filenames)
{
	DIR* pDir;
	struct dirent* ptr;
	if (!(pDir = opendir(dir.c_str())))
		return;
	while ((ptr = readdir(pDir)) != 0) {
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
			filenames.push_back(dir + "/" + ptr->d_name);
	}
	closedir(pDir);
}
#elif _WIN32
void ManagerLog::listFiles(std::string dir, std::vector<std::string>& folderlist)
{
	std::string tmpdir = dir + std::string("*.*");
	intptr_t handle;
	_finddata_t findData;
	handle = _findfirst(tmpdir.c_str(), &findData);// 查找目录中的第一个文件
	if (handle == -1)
	{
		return;
	}
	do
	{
		if (findData.attrib & _A_SUBDIR && strcmp(findData.name, ".") != 0 && strcmp(findData.name, "..") != 0)
			folderlist.push_back(findData.name);

	} while (_findnext(handle, &findData) == 0);

	_findclose(handle);// 关闭搜索句柄
}
#endif

std::string ManagerLog::getCurrentTimeString()
{
	time_t now = time(0);
	struct tm tstruct;

#ifdef _WIN32
	localtime_s(&tstruct, &now);
#else
	localtime_r(&now, &tstruct);
#endif

	char buf[80];
	snprintf(buf, sizeof(buf), "%04d_%02d_%02d_%02d_%02d_%02d",
		tstruct.tm_year + 1900,
		tstruct.tm_mon + 1,
		tstruct.tm_mday,
		tstruct.tm_hour,
		tstruct.tm_min,
		tstruct.tm_sec);
	return std::string(buf);
}

std::string ManagerLog::getCurrentTime()
{
	time_t now = time(NULL);
	struct tm tm;
#ifdef _WIN32
	localtime_s(&tm, &now);
#else
	localtime_r(&now, &tm);
#endif
	char result[64] = { 0 };
	sprintf(result, "%04d%02d%02d_%02d%02d%02d", 1900 + tm.tm_year, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	return std::string(result);
}

std::string ManagerLog::getCurrentTimeWithMs()
{
#ifdef _WIN32
	SYSTEMTIME st;
	GetLocalTime(&st);
	char result[64] = { 0 };
	sprintf(result, "%04d%02d%02d_%02d%02d%02d.%03d",
		st.wYear, st.wMonth, st.wDay,
		st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	return std::string(result);
#else
	struct timespec ts;
	struct tm tm;
	clock_gettime(CLOCK_REALTIME, &ts);
	localtime_r(&ts.tv_sec, &tm);
	char result[64] = { 0 };
	sprintf(result, "%04d%02d%02d_%02d%02d%02d.%03ld",
		1900 + tm.tm_year, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000);
	return std::string(result);
#endif
}

std::string ManagerLog::getExecutablePath()
{
#ifdef _WIN32
	char buffer[MAX_PATH];
	GetModuleFileNameA(NULL, buffer, MAX_PATH);
	std::string path(buffer);
	size_t pos = path.find_last_of("\\/");
	if (pos != std::string::npos) {
		return path.substr(0, pos + 1);
	}
	return path;
#else
	char buffer[PATH_MAX];
	ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
	if (len != -1) {
		buffer[len] = '\0';
		std::string path(buffer);
		size_t pos = path.find_last_of("/");
		if (pos != std::string::npos) {
			return path.substr(0, pos + 1);
		}
		return path;
	}
	return "./";
#endif
}

bool ManagerLog::IsDirWritable(const std::string& path)
{
    char filename[256];
    snprintf(filename, sizeof(filename), 
             "%s%swrite_test_%u_%d.tmp",
             path.c_str(), 
             PATH_SEPARATOR,
             (unsigned)time(NULL),
             GETPID());

	// printf("%s\n", filename);		 

    FILE* fp = fopen(filename, "w");
    if (fp) {
        fclose(fp);
        remove(filename);
        return true;
    }
    return false;
}

uint64_t ManagerLog::getDiskFreeSpace(const std::string &path)
{
#ifdef _WIN32
	// Windows: 使用 GetDiskFreeSpaceEx
	std::string rootPath = path;

	// 提取根目录
	if (path.length() >= 2 && path[1] == ':')
	{
		rootPath = path.substr(0, 3); // 例如 "C:\"
	}
	else if (path.length() >= 1 && (path[0] == '\\' || path[0] == '/'))
	{
		// UNC 路径 \\server\share 或 //server/share
		size_t pos = path.find_first_of("\\/", 2);
		if (pos != std::string::npos)
		{
			pos = path.find_first_of("\\/", pos + 1);
			if (pos != std::string::npos)
			{
				rootPath = path.substr(0, pos); // \\server\share
			}
		}
	}

	ULARGE_INTEGER freeBytesAvailable;
	if (GetDiskFreeSpaceExA(rootPath.c_str(), &freeBytesAvailable, NULL, NULL))
	{
		return freeBytesAvailable.QuadPart;
	}
	return 0;
#else
	// Linux: 逐级向上找存在的目录
	std::string checkPath = path;

	// 统一分隔符为 '/'
	for (size_t i = 0; i < checkPath.length(); i++) {
		if (checkPath[i] == '\\') checkPath[i] = '/';
	}

	// 去除末尾的 '/'（根目录除外），避免多遍历一次
	if (checkPath.length() > 1 && checkPath[checkPath.length() - 1] == '/') {
		checkPath.erase(checkPath.length() - 1);
	}

	// 逐级向上查找存在的目录
	while (!checkPath.empty()) {
		struct statvfs svfs;
		if (statvfs(checkPath.c_str(), &svfs) == 0) {
			return (uint64_t)svfs.f_bavail * svfs.f_frsize;
		}

		// 向上找父目录
		size_t pos = checkPath.find_last_of('/');
		if (pos == std::string::npos) break;

		checkPath = checkPath.substr(0, pos);
		// 如果截断后为空，说明已经到根目录了
		if (checkPath.empty()) {
			checkPath = "/";
		}
	}

	return 0;
#endif
}

bool ManagerLog::checkDiskSpace(const std::string& path)
{
	uint64_t freeSpace = getDiskFreeSpace(path);
	uint64_t minSpaceBytes = m_minDiskSpaceMB * 1024 * 1024;
	return freeSpace >= minSpaceBytes;
}

bool ManagerLog::saveLog(bool isSave)
{
	if (isSave) {
		// 优先使用用户设置的路径，否则使用可执行文件路径
		std::string Logpath = m_custom_log_path.empty() ? getExecutablePath() : m_custom_log_path;

		// 确保路径以分隔符结尾
		if (!Logpath.empty() && Logpath[Logpath.length() - 1] != '\\' && Logpath[Logpath.length() - 1] != '/') {
			Logpath += PATH_SEPARATOR;
		}

		// 检查写权限
		if (!IsDirWritable(Logpath)) {
			// 如果没有写权限，尝试使用当前工作目录
			std::string failedPath = Logpath;
#ifdef _WIN32
			char cwd[MAX_PATH];
			if (_getcwd(cwd, MAX_PATH) != NULL) {
				Logpath = std::string(cwd) + PATH_SEPARATOR;
			}
#else
			char cwd[PATH_MAX];
			if (getcwd(cwd, sizeof(cwd)) != NULL) {
				Logpath = std::string(cwd) + PATH_SEPARATOR;
			}
#endif
			printf("Log path '%s' not writable, trying current dir '%s'\n", failedPath.c_str(), Logpath.c_str());
			if (!IsDirWritable(Logpath)) {
				printf("Current dir not writable either.\n");
				return false;
			}
		}

		// 构建日志子目录
		std::string logPath2 = std::string("PaceLog") + PATH_SEPARATOR + getCurrentTimeString() + PATH_SEPARATOR;
		Logpath += logPath2;

		// 初始化时检查磁盘空间
		if (!checkDiskSpace(Logpath)) {
			printf("Disk full, Minimum required: %llu MB\n",
				(unsigned long long)m_minDiskSpaceMB);
			return false;
		}

		// 创建目录
		if (!MkDir(Logpath)) {
			printf("Failed to create log directory: %s\n", Logpath.c_str());
			return false;
		}

		std::string file3 = "data";

		// 调用ManagerLog初始化
		if (!init(Logpath, file3, 1)) {
			return false;
		}
	}
	else {
		uninit();
	}
	return true;
}

// 自动日志控制：检测到错误时调用
bool ManagerLog::autoStartLog()
{
	// 如果未启用，直接返回
	if (!m_enabled) {
		return false;
	}

	lock();

	// 已经在记录，重置稳定等待状态（因为又出错了）-> 时间后移	
	if (m_logAutoStarted) {
		if (m_waitingForStable) {
			printf("[%s] Packet lost during stable period\n", getCurrentTimeWithMs().c_str());
			m_waitingForStable = false;	// 重置稳定期标志
		}
		unlock();
		return true;
	}

	if (!saveLog(true)) {
		printf("Failed to start log\n");
		unlock();
		return false;
	}

	m_logAutoStarted = true;
	m_waitingForStable = false;
	printf("[%s] Auto start log\n", getCurrentTimeWithMs().c_str());
	unlock();
	return true;
}

// 自动日志控制：数据正常时调用，内部处理3秒稳定期
void ManagerLog::autoCheckStopLog()
{
	// 如果未启用，直接返回
	if (!m_enabled) {
		return;
	}

	lock();

	if (!m_logAutoStarted) {
		unlock();
		return;
	}

#ifdef _WIN32
	uint64_t now = GetTickCount64();
#else
	timespec now_ts;
	clock_gettime(CLOCK_MONOTONIC, &now_ts);
#endif

	if (!m_waitingForStable) {
		printf("[%s] Data stable\n", getCurrentTimeWithMs().c_str());
		m_waitingForStable = true;
#ifdef _WIN32
		m_firstStableTime = now;
#else
		m_firstStableTime = now_ts;
#endif
		unlock();
		return;
	}

	// 检查是否满3秒
#ifdef _WIN32
	uint64_t elapsed = now - m_firstStableTime;
#else
	uint64_t elapsed_ms = (uint64_t)((now_ts.tv_sec - m_firstStableTime.tv_sec) * 1000 +
		(now_ts.tv_nsec - m_firstStableTime.tv_nsec) / 1000000);
#endif

#ifdef _WIN32
	if (elapsed >= 3000) {
#else
	if (elapsed_ms >= 3000) {
#endif
		saveLog(false);
		m_logAutoStarted = false;
		m_waitingForStable = false;
		printf("[%s] Auto stop log\n", getCurrentTimeWithMs().c_str());
	}
	unlock();
}

// 自动日志控制：线程退出时清理
void ManagerLog::autoLogCleanup()
{
	lock();

	if (m_logAutoStarted) {

		saveLog(false);
		m_logAutoStarted = false;
		m_waitingForStable = false;
		printf("Clean log\n");
	}
	unlock();
}