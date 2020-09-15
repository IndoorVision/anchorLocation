
#include "Runtime.h"





CRuntime::CRuntime(std::string s)
{
	str = s;
	t_start = getTimeStamp();
}


CRuntime::~CRuntime()
{
	t_end = getTimeStamp();
	//printf(">>>%s waste time %d ms\n", str.c_str(), t_end - t_start);
	std::cout << str << " ms " << t_end - t_start << std::endl;
}
