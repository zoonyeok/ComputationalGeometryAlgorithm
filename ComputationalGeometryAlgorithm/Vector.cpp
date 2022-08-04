#include "Vector.h"


float zoonyeok::CrossProduct2D(Vector2f v1, Vector2f v2)
{
	return v1[X] * v2[Y] - v1[Y] * v2[X];
}

zoonyeok::Vector3f zoonyeok::CrossProduct3D(Vector3f a, Vector3f b)
{
	float x_, y_, z_;
	x_ = a[Y] * b[Z] - b[Y] * a[Z];
	y_ = -(b[Z] * a[X] - a[Z] * b[X]);
	z_ = a[X] * b[Y] - b[X] * a[Y];

	return Vector3f(x_, y_, z_);
}

float zoonyeok::ScalarTripleProduct(Vector3f v1, Vector3f v2, Vector3f v3)
{
	Vector3f cross = CrossProduct3D(v2, v3);
	return DotProduct(v1, cross);
}
