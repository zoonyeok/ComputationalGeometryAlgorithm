#include "intersection.h"

// simply check whether the two line segments are intersecting on
bool zoonyeok::Intersection(const Point2d& a, const Point2d& b, const Point2d& c, const Point2d& d)
{
	auto ab_c = orientation2D(a, b, c);
	auto ab_d = orientation2D(a, b, d);
	auto cd_a = orientation2D(c, d, a);
	auto cd_b = orientation2D(c, d, b);

	if (ab_c == BETWEEN || ab_c == ORIGIN || ab_c == DESTINATION
		|| ab_d == BETWEEN || ab_d == ORIGIN || ab_d == DESTINATION
		|| cd_a == BETWEEN || cd_a == ORIGIN || cd_a == DESTINATION
		|| cd_b == BETWEEN || cd_b == ORIGIN || cd_b == DESTINATION)
	{
		return true;
	}

	return _xor(ab_c == LEFT, ab_d == LEFT) && _xor(cd_a == LEFT, cd_b == LEFT);
}

bool zoonyeok::Intersection(const Point2d& a, const Point2d& b, const Point2d& c, const Point2d& d, Point2d& intersection)
{
	Vector2f AB = b - a;
	Vector2f CD = d - c;

	Vector2f normal (CD[Y], - CD[X]);

	auto denominator = DotProduct(normal, AB);
	if (isEqualD(denominator, ZERO))
	{
		auto AC = c - a;
		auto numerator = DotProduct(normal, AC);

		auto t = numerator / denominator;

		auto x = a[X] + t * AB[X];
		auto y = a[Y] + t * AB[Y];

		intersection.Assign(X, x);
		intersection.Assign(Y, y);
		return true;
	}
	return false;
}

bool zoonyeok::Intersection(const Line2d& l1, const Line2d& l2, Point2d& intersection)
{
	Vector2f l1_start = l1.GetPoint();
	Vector2f l1_end = l1_start + l1.GetDirection();
	Vector2f l2_start = l2.GetPoint();
	Vector2f l2_end = l2_start + l2.GetDirection();

	return Intersection(l1_start, l1_end, l2_start, l2_end, intersection);
}

bool zoonyeok::Intersection(const Line3d& line, const Planef& plane, Point3d& point)
{
	Vector3f n = plane.GetNormal();
	float D = plane.DistancePointToPlane();
	Vector3f d = line.GetDirection();
	Vector3f p = line.GetPoint();
	float dotnd = DotProduct(n, d);

	if (IsNearlyEqual(dotnd, ZERO))
	{
		return false;
	}

	float t = (-1 * DotProduct(n, p) + D) / dotnd;
	point.Assign(X, p[X] + t * d[X]);
	point.Assign(Y, p[Y] + t * d[Y]);
	point.Assign(Z, p[Z] + t * d[Z]);
	return true;
}

bool zoonyeok::Intersection(const Planef& plane1, Planef& plane2, Line3d& line)
{
	Vector3f n1 = plane1.GetNormal();
	Vector3f n2 = plane2.GetNormal();
	float d1 = plane1.DistancePointToPlane();
	float d2 = plane2.DistancePointToPlane();

	Vector3f direction = CrossProduct3D(n1, n2);

	// check if the planes are parallel.
	if (IsNearlyEqual(direction.Magnitude(), ZERO)) return false;

	float n1n2 = DotProduct(n1, n2);
	float n1n2_2 = n1n2 * n1n2;

	float a = (d2 * n1n2 - d1) / (n1n2_2 - 1);
	float b = (d1 * n1n2 - d2) / (n1n2_2 - 1);

	Vector3f point = n1 * a + n2 * b;

	line.SetPoint(point);
	direction.Normalize();
	line.SetDirection(direction);

	return true;
}