#include "winUtil.hpp"

float imaiUtil::getTimeDiff()
{
	static LARGE_INTEGER qpcFreq;	QueryPerformanceFrequency(&qpcFreq);
	static LARGE_INTEGER qpcNow;	//qpcNow.QuadPart = 0;
	static LARGE_INTEGER qpcPre;	//qpcPre.QuadPart = 0;
		
	qpcPre.QuadPart = qpcNow.QuadPart;
	QueryPerformanceCounter(&qpcNow);
	return( (float)(qpcNow.QuadPart-qpcPre.QuadPart) / (float)(qpcFreq.QuadPart/1000.0) );
}

