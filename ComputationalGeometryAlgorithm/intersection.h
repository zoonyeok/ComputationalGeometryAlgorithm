#pragma once
#include "Point.h"
#include "intersection.h"
#include "GeoUtils.h"
#include "Core.h"
#include "Line.h"
#include "Plane.h"

namespace zoonyeok
{
	
	// simply check whether the two line segments are intersecting on
	bool Intersection(const Point2d& a, const Point2d& b, const Point2d& c,const Point2d& d);
	
	bool Intersection(const Point2d& a, const Point2d& b, const Point2d& c, const Point2d& d, Point2d& intersection);

	bool Intersection(const Line2d& a, const Line2d& b, Point2d& intersection);

	bool Intersection(const Line3d& line, const Planef& plane, Point3d& point);

	bool Intersection(const Planef& plane1, Planef& plane2, Line3d& line);

}