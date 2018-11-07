#include "MyLogger.h"


MyLogger::MyLogger()
{}

MyLogger::MyLogger(string logFileName)
{
	m_logFileName = logFileName;
	m_log.open(m_logFileName);
}

void MyLogger::appendLog(string logmsg)
{
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	m_log << ms.count() << " " << logmsg << endl;
}

void MyLogger::appendLog(int logmsg)
{
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	m_log << ms.count() << " " << logmsg << endl;
}

void MyLogger::appendLog(float logmsg)
{
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	m_log << ms.count() << " " << logmsg << endl;
}
