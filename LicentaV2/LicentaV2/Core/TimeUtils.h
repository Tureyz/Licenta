#pragma once

#include <chrono>

namespace TimeUtils
{
	static const float eps = 0.00000001f;
	typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> TimePointNano;

	TimePointNano Now();
	long long DurationNano(TimePointNano lhs, TimePointNano rhs);
	float NToS(long long nanos);
	float NToMs(long long nanos);
	float Duration(TimePointNano lhs, TimePointNano rhs);
}