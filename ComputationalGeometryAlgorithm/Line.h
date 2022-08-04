#pragma once
#include "Vector.h"

namespace zoonyeok
{
	template<class coord_type, size_t dim = DIM3>
	class Line 
	{
		Vector<coord_type, dim> point;
		Vector<coord_type, dim> dir;

	public:
		Line() {}

		Line(Vector<coord_type,dim>& p1, Vector<coord_type,dim>& p2) 
		{
			dir = p2 - p1;
			dir.Normalize();
			point = p1;
		}

		Vector<coord_type, dim> GetPoint() const;

		Vector<coord_type, dim> GetDirection() const;

		void SetPoint(const Vector<coord_type, dim>& inputPoint);

		void SetDirection(const Vector<coord_type, dim>& inputDirection);
	};

	typedef Line<float, DIM2> Line2d;
	typedef Line<float, DIM3> Line3d;

	template<class coord_type, size_t dim /*= DIM3*/>
	zoonyeok::Vector<coord_type, dim> zoonyeok::Line<coord_type, dim>::GetDirection() const
	{
		return dir;
	}

	template<class coord_type, size_t dim /*= DIM3*/>
	zoonyeok::Vector<coord_type, dim> zoonyeok::Line<coord_type, dim>::GetPoint() const
	{
		return point;
	}

	template<class coord_type, size_t dim /*= DIM3*/>
	void zoonyeok::Line<coord_type, dim>::SetDirection(const Vector<coord_type, dim>& inputDirection)
	{
		direction = inputDirection;
	}

	template<class coord_type, size_t dim /*= DIM3*/>
	void zoonyeok::Line<coord_type, dim>::SetPoint(const Vector<coord_type, dim>& inputPoint)
	{
		point = inputDirection;
	}
}
