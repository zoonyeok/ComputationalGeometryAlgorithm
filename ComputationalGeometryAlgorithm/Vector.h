#pragma once

#include <array>
#include <iostream>
#include "Core.h"

namespace zoonyeok
{

#define DIM2 2
#define DIM3 3

#define X	0
#define Y	1
#define Z	2

	template<class coordinate_type, size_t dimensions = DIM3>
	class Vector
	{
		static_assert(std::is_arithmetic_v<coordinate_type>, "Vector class can only store Integral or Floating point values");
		static_assert(dimensions >= DIM2, "Vector dimension at least should be 2D");

		std::array<coordinate_type, dimensions> coords;

		template<class coordinate_type, size_t dimensions>
		friend float DotProduct(const Vector<coordinate_type, dimensions>& v1, const Vector<coordinate_type, dimensions>& v2);

	public:
		Vector() {};

		Vector(std::array<coordinate_type, dimensions> _coords) :coords(_coords) { }

		Vector(coordinate_type _x, coordinate_type _y, coordinate_type _z) :coords(_x, _y, _z) { }

		Vector(coordinate_type _x, coordinate_type _y) : coords({ _x,_y }) {}

		// Equality check
		bool operator==(const Vector<coordinate_type, dimensions>&) const;

		// Not equal
		bool operator!=(const Vector<coordinate_type, dimensions>&) const;

		// Addition
		Vector<coordinate_type, dimensions> operator+(const Vector<coordinate_type, dimensions>&) const;

		// Subtraction
		Vector<coordinate_type, dimensions> operator-(const Vector<coordinate_type, dimensions>&) const;

		// Multi
		Vector<coordinate_type, dimensions> operator*(coordinate_type value);

		// Less than operator
		Vector<coordinate_type, dimensions> operator<(const Vector<coordinate_type, dimensions>&) const;

		// Greater than operator
		Vector<coordinate_type, dimensions> operator>(const Vector<coordinate_type, dimensions>&) const;

		coordinate_type operator[](int) const;

		void Assign(int dim, coordinate_type value);

		float Magnitude() const;

		void Normalize();
	};

	typedef Vector<float, DIM2> Vector2f;
	typedef Vector<float, DIM3> Vector3f;

	template<class coordinate_type, size_t dimensions>
	inline bool Vector<coordinate_type, dimensions>::operator==(const Vector<coordinate_type, dimensions>& _other) const
	{
		for (int i = 0; i < dimensions; i++)
		{
			if (IsNearlyEqual(coords[i], _other.coords[i]))
				return false;
		}
		return true;
	}
	template<class coordinate_type, size_t dimensions>
	inline bool Vector<coordinate_type, dimensions>::operator!=(const Vector<coordinate_type, dimensions>& _other) const
	{
		return !(*this == _other);
	}

	template<class coordinate_type, size_t dimensions>
	inline Vector<coordinate_type, dimensions> Vector<coordinate_type, dimensions>::operator+(const Vector<coordinate_type, dimensions>& _other) const
	{
		std::array<coordinate_type, dimensions> tmp_array;
		for (size_t i = 0; i < dimensions; i++)
		{
			tmp_array[i] = coords[i] + _other.coords[i];
		}
		return Vector<coordinate_type, dimensions>(tmp_array);
	}

	template<class coordinate_type, size_t dimensions>
	inline Vector<coordinate_type, dimensions> Vector<coordinate_type, dimensions>::operator-(const Vector<coordinate_type, dimensions>& _other) const
	{
		std::array<coordinate_type, dimensions> tmp_array;
		for (size_t i = 0; i < dimensions; i++)
		{
			tmp_array[i] = coords[i] - _other.coords[i];
		}
		return Vector<coordinate_type, dimensions>(tmp_array);
	}

	template<class coordinate_type, size_t dimensions>
	inline Vector<coordinate_type, dimensions> Vector<coordinate_type, dimensions>::operator*(coordinate_type value)
	{
		std::array<coordinate_type, dimensions> temp_array;
		for (int i = 0; i < dimensions; i++)
		{
			temp_array[i] = coords[i] * value;
		}

		return Vector<coordinate_type, dimensions>(temp_array);
	}

	template<class coordinate_type, size_t dimensions>
	inline Vector<coordinate_type, dimensions> Vector<coordinate_type, dimensions>::operator<(const Vector<coordinate_type, dimensions>& _other) const
	{
		for (size_t i = 0; i < dimension; i++)
		{
			if (this->coords[i] < _other.coords[i])
				return true;
			else if (this->coords[i] > _other.coords[i])
				return false;
		}
		return false;
	}

	template<class coordinate_type, size_t dimensions>
	inline Vector<coordinate_type, dimensions> Vector<coordinate_type, dimensions>::operator>(const Vector<coordinate_type, dimensions>& _other) const
	{
		for (size_t i = 0; i < dimension; i++)
		{
			if (this->coords[i] > _other.coords[i])
				return true;
			else if (this->coords[i] < _other.coords[i])
				return false;
		}
		return false;
	}

	template<class coordinate_type, size_t dimensions>
	inline coordinate_type Vector<coordinate_type, dimensions>::operator[](int _index) const
	{
		if (_index >= coords.size())
		{
			std::cout << "Index out of bound" << endl;
			return coordinate_type();
		}
		return coord[_index];
	}

	template<class coordinate_type, size_t dimensions>
	inline void Vector<coordinate_type, dimensions>::Assign(int dim, coordinate_type value)
	{
		if (_index >= coords.size())
		{
			std::cout << "Index out of bound" << endl;
			return coordinate_type();
		}

		coord[dim] = value;
	}

	template<class coordinate_type, size_t dimensions>
	inline float Vector<coordinate_type, dimensions>::Magnitude() const
	{
		float value = 0.0f;

		for (size_t i = 0; i < dimensions; i++)
		{
			value += pow(coords[i], 2.0f);
		}

		return sqrt(value);
	}

	template<class coordinate_type, size_t dimensions>
	inline void Vector<coordinate_type, dimensions>::Normalize()
	{
		auto mag = Magnitude();

		for (size_t i = 0; i < dimensions; i++)
		{
			assign(i, coords[i] / mag);
		}
	}

	template<class coordinate_type, size_t dimensions>
	float DotProduct(const Vector<coordinate_type, dimensions>& v1, const Vector<coordinate_type, dimensions>& v2)
	{
		if (v1.coords.size() != v2.coords.size())
			return FLT_MIN;

		float product = 0.0f;

		for (size_t i = 0; i < dimensions; i++)
		{
			product += v1[i] * v2[i];
		}

		return product;
	}

	float CrossProduct2D(Vector2f v1, Vector2f v2);

	Vector3f CrossProduct3D(Vector3f v1, Vector3f v2);

	float ScalarTripleProduct(Vector3f v1, Vector3f v2, Vector3f v3);
}