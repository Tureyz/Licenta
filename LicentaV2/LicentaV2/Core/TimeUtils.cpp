#include "TimeUtils.h"

TimeUtils::TimePointNano TimeUtils::Now()
{
	return std::chrono::high_resolution_clock::now();
}

long long TimeUtils::DurationNano(TimePointNano lhs, TimePointNano rhs)
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(rhs - lhs).count();
}

float TimeUtils::NToS(long long nanos)
{
	return static_cast<float>(nanos) / 1000000000.f;
}

float TimeUtils::NToMs(long long nanos)
{
	return static_cast<float>(nanos) / 1000000.f;
}

float TimeUtils::Duration(TimePointNano lhs, TimePointNano rhs)
{
	return NToS(DurationNano(lhs, rhs));
}

float TimeUtils::DurationMS(TimePointNano lhs, TimePointNano rhs)
{
	return NToMs(DurationNano(lhs, rhs));
}

