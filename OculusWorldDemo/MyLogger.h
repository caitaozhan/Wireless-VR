#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>
using namespace std;
using namespace chrono;

class MyLogger
{
public:
	MyLogger();
	MyLogger(string logFileName);

	void appendLog(string logmsg);  // append log
	void appendLog(int logmsg);     // append log

private:
	string m_logFileName;
	ofstream m_log;
};
