#include "Distance.h"

float zoonyeok::distance(Line3d& line, Point3d& C)
{
	Vector3f AC = C - line.GetPoint();
	float t = DotProduct(line.GetDirection(), AC);

	Vector3f xt = line.GetPoint() + line.GetDirection() * t;
	Vector3f dist_vec = xt - C;

	return dist_vec.Magnitude();
}

float zoonyeok::distance(Planef& p, Point3d& Q)
{
	float result = DotProduct(p.GetNormal(), Q) - p.DistancePointToPlane();
	return result;
}
