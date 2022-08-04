#include "GeoUtils.h"

double zoonyeok::areaTriangle2D(const Point2d& a, const Point2d& b, const Point2d& c)
{
	auto AB = b - a;
	auto AC = c - a;
	auto result = CrossProduct2D(AB, AC);
	return result / 2;
}

int zoonyeok::orientation2D(const Point2d& a, const Point2d& b, const Point2d& c)
{
	auto area = areaTriangle2D(a, b, c);

	if (area > 0 && area < TOLERANCE)
		area = 0;

	if (area < 0 && area > TOLERANCE)
		area = 0;

	Vector2f ab = b - a;
	Vector2f ac = c - a;

	if (area > 0)
		return LEFT;
	if (area < 0)
		return RIGHT;
	if ((ab[X] * ac[X] < 0) || (ab[Y] * ac[Y] < 0))
		return BEHIND;
	if (ab.Magnitude() < ac.Magnitude())
		return BEYOND;
	if (a == c)
		return ORIGIN;
	if (b == c)
		return DESTINATION;

	return BETWEEN;
}

bool zoonyeok::Iscollinear(const Vector3f& a, const Vector3f& b)
{
	float v1 = (a[X] * b[Y] - a[Y] * b[X]);
	float v2 = (a[Y] * b[Z] - a[Z] * b[Y]);
	float v3 = (a[X] * b[Z] - a[Z] * b[X]);

	return IsNearlyEqual(v1, ZERO) && IsNearlyEqual(v2, ZERO) && IsNearlyEqual(v3, ZERO);
}


bool zoonyeok::Iscollinear(const Point3d& a, const Point3d& b, const Point3d& c)
{
	Vector3f AB = b - a;
	Vector3f AC = c - a;
	return Iscollinear(AB, AC);
}

// 스칼라삼중곱이 0이 될때만 부피의 값이 0이 되므로 명시적으로 1/6을 해줄 필요없음
bool zoonyeok::IsCoplanar(const Vector3f a, const Vector3f b, const Vector3f c)
{
	float scalarTripleProduct = ScalarTripleProduct(a, b, c);
	return IsNearlyEqual(scalarTripleProduct, ZERO);
}

bool zoonyeok::IsCoplanar(const Point3d& a, const Point3d& b, const Point3d& c, const Point3d& d)
{
	Vector3f AB = b - a;
	Vector3f AC = c - a;
	Vector3f AD = d - a;
	return IsCoplanar(AB, AC, AD);
}