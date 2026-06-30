#ifndef __H_MTIME_H__
#define __H_MTIME_H__

#include <chrono>
#include <unistd.h>
#include <time.h>

enum TIME_PRECISION
{
	TIME_NANOSECOND = 0,	//Nanosecond
	TIME_MICROSECOND,		//Microsecond
	TIME_MILLISECOND,		//Millisecond
	TIME_SECOND,
	TIME_MINUTE,
	TIME_HOUR
};

//Calculate elapsed time
extern std::chrono::time_point<std::chrono::steady_clock> time_start;
inline void timestart()
{
	time_start = std::chrono::steady_clock::now();
}
inline int64_t timeused()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
}
inline int64_t timeused(std::chrono::time_point<std::chrono::steady_clock>& time_last)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_last).count();
}

//Sleep for how many milliseconds
void sleep_ms(int ms);

//Get tm (only update year, month, day, hour, minute, second)
struct tm localdate(int time_zone = 8);

//Get time [integer]
int64_t current_times(int precision = TIME_MILLISECOND);

//Get time [string]
char *time_str(const char *fmt = "%F %T");

#endif
