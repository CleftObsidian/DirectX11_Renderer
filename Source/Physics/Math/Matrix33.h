#pragma once

#include "Vector3D.h"

class Matrix33 final
{
public:
	Matrix33();
	Matrix33(DOUBLE a_values[9]);
	~Matrix33();
	
	Matrix33 Invert() const;
	Vector3D operator*(const Vector3D& vector) const;
	Matrix33 operator*(const Matrix33& matrix) const;
	Matrix33 operator*(const DOUBLE scalar) const;
	Matrix33 operator+(const Matrix33& matrix) const;

	static Matrix33 Skew(Vector3D vector);
	static Vector3D NewtonSolve(Vector3D vector, Vector3D fV, const Matrix33& jacobian);

public:
	DOUBLE m_aaElements[3][3];
};