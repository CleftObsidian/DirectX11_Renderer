#pragma once

#include "Common.h"

class Vector3D final
{
public:
	Vector3D();
	Vector3D(DOUBLE x, DOUBLE y, DOUBLE z);
	Vector3D(Vector3D vectorStart, Vector3D vectorEnd);
	~Vector3D();

	Vector3D GetInverse();
	Vector3D GetUnitVector() const;
	DOUBLE GetMagnitude() const;
	DOUBLE GetMagnitudeSquared() const;

	DOUBLE DotProduct(const Vector3D vector) const;
	Vector3D CrossProduct(const Vector3D vector) const;
	Vector3D Add(const Vector3D vector) const;
	Vector3D operator+(const Vector3D& vector) const;
	Vector3D Sub(const Vector3D vector) const;
	Vector3D operator-(const Vector3D& vector) const;
	Vector3D ScalarMultiply(DOUBLE scalar) const;
	Vector3D operator*(DOUBLE scalar) const;

	BOOL IsZero() const;
	BOOL operator!=(const Vector3D& vector);
	BOOL operator==(const Vector3D& vector);

public:
	DOUBLE m_x;
	DOUBLE m_y;
	DOUBLE m_z;
};