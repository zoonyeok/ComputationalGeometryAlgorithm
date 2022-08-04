#pragma once
#include "Vector.h"
#include "Point.h"

namespace zoonyeok
{
	template<class coord_type>
	class Plane
	{
		Vector3f normal;
		float d = 0;

	public:
		Plane(){}

		Plane(Vector3f& _normal, float _constant) : normal(_normal.Normalize()), d(_constant) {}

		Plane(Point3d& _p1, Point3d& _p2, Point3d& _p3)
		{
			auto v12 = _p2 - _p1;
			auto v13 = _p3 - _p1;

			normal = CrossProduct2D(v12, v13);
			normal.Normalize();
			d = dotProduct(normal, _p1);
		}

		Vector3f GetNormal() const { return normal; }

		//return the distance from point to a given plane
		float DistancePointToPlane() const { return d; }
	};
	typedef Plane<float> Planef;

	
}