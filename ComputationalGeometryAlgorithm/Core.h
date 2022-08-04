#pragma once
#include <math.h>

#define TOLERANCE 0.0000001

#define ZERO 0

#define PI 					(3.1415926535897932f)
#define SMALL_NUMBER		(1.e-8f)

enum ERELATIVE_POSITON
{
	LEFT, RIGHT, BEHIND, BEYOND, BETWEEN, ORIGIN, DESTINATION
};

template< class T >
static constexpr inline T Abs(const T A)
{
	return (A >= (T)0) ? A : -A;
}

static inline bool isEqualD(double x, double y)
{
	return fabs(x - y) < TOLERANCE;
}

static inline bool _xor(bool x, bool y)
{
	return x ^ y;
}

template<class T>
static inline auto RadiansToDegrees(T const& RadVal) -> decltype(RadVal* (180.f / PI))
{
	return RadVal * (180.f / PI);
}

static inline bool IsNearlyEqual(double A, double B, double ErrorTolerance = SMALL_NUMBER)
{
	return Abs<double>(A - B) <= ErrorTolerance;
}