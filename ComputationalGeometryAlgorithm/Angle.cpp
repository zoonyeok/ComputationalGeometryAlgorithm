#include "Angle.h"

float zoonyeok::AngleLines2D(const Line2d& l1, const Line2d& l2)
{
	return GetAngle(l1.GetDirection(), l2.GetDirection());
}

float zoonyeok::AngleLines3D(const Line3d& l1, const Line3d& l2)
{
	return GetAngle(l1.GetDirection(), l2.GetDirection());
}

float zoonyeok::AngleLinePlane(const Line3d& l1, const Planef& p)
{
	return 90.f - GetAngle(l1.GetDirection(), p.GetNormal());
}

float zoonyeok::AnglePlanes(const Planef& p1, const Planef& p2)
{
	return GetAngle(p1.GetNormal(), p2.GetNormal());
}

template<class T, size_t dimension>
static float GetAngle( zoonyeok::Vector<T, dimension> v1, zoonyeok::Vector<T, dimension> v2)
{
	auto dot = DotProduct(v1, v2);
	auto angle = acos(fabs(dot));
	return RadiansToDegrees(angle);
}