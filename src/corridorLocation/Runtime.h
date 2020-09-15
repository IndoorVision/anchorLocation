#pragma once
#include <iostream>
#include <string>
#include <cstdlib>
#include <time.h>
#include <ctime>
#include <chrono>

class CRuntime
{
	std::time_t t_start;
	std::time_t t_end;
	std::string str;
public:
	CRuntime(std::string s ="");
	~CRuntime();
	std::time_t getTimeStamp()
	{
		std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
		auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
		std::time_t timestamp = tmp.count();
		//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
		return timestamp;
	}
};

