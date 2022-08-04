#pragma once

#include "Point.h"
#include "Line.h"
#include "Plane.h"

namespace zoonyeok
{
	float distance(Line3d& line, Point3d& C);

	float distance(Planef& p, Point3d& Q);
}