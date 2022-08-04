#pragma once
#include "Line.h"
#include "Plane.h"

namespace zoonyeok
{
	float AngleLines2D(const Line2d& l1, const Line2d& l2);

	float AngleLines3D(const Line3d& l1, const Line3d& l2);

	float AngleLinePlane(const Line3d& l1, const Planef& p);
	
	float AnglePlanes(const Planef& p1, const Planef& p2);
}