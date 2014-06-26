/******************************************************
NTU EECS Advanced Control Lab, Intelligent Robot Lab

2010 Kuo-Chen Huang

******************************************************/

#ifndef MSLEEP_H
#define MSLEEP_H

#include <time.h>

#ifdef __linux__

inline int __nsleep( const struct timespec *req )
{
	struct timespec rem;
	struct timespec temp_req = *req;
	while( nanosleep(&temp_req, &rem) == -1 ){
		temp_req = rem;
	}
	return 1;
}

inline int msleep(unsigned long milisec)
{
	struct timespec req = {0};
	time_t sec = milisec / 1000;
	milisec = milisec % 1000;
	req.tv_sec = sec;
	req.tv_nsec = milisec * 1000000L;
	__nsleep( &req );
	return 1;
}

#endif

#endif
