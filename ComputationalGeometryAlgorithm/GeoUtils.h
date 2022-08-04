#pragma once
#include "Point.h"
#include "Core.h"

namespace zoonyeok
{
	double areaTriangle2D(const Point2d& a, const Point2d& b, const Point2d& c);

	int orientation2D(const Point2d& a, const Point2d& b, const Point2d& c);

	bool Iscollinear(const Vector3f& a, const Vector3f& b);

	bool Iscollinear(const Point3d& a, const Point3d& b, const Point3d& c);

	bool IsCoplanar(const Vector3f a, const Vector3f b, const Vector3f c);

	bool IsCoplanar(const Point3d& a, const Point3d& b, const Point3d& c, const Point3d& d);

	// Predicate to determine whether the [Point c] is left to the segment [a b]
	bool left(const Point3d& a, const Point3d& b, const Point3d& c);

	// Predicate to determine whether the [Point c] is left to the segment [a b]
	bool left(const Point2d& a, const Point2d& b, const Point2d& c);

	// Predicate to determine whether the [Point c] is right to the segment [a b]
	bool right(const Point3d& a, const Point3d& b, const Point3d& c);

	// Predicate to determine whether the[Point c] is left to the segment[a b]
	bool leftOrBeyond(const Point2d& a, const Point2d& b, const Point2d& c);

	// Predicate to determine whether the [Point c] is left to the segment [a b]
	bool leftOrBeyond(const Point3d& a, const Point3d& b, const Point3d& c);

	// Predicate to determine whether the [Point c] is left to or between the segment [a b]
	bool leftOrBetween(const Point3d& a, const Point3d& b, const Point3d& c);

}