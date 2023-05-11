#include "Vector3D.h"

Vector3D::Vector3D()
	: m_x(0.0)
	, m_y(0.0)
	, m_z(0.0)
{
}

Vector3D::Vector3D(DOUBLE x, DOUBLE y, DOUBLE z)
	: m_x(x)
	, m_y(y)
	, m_z(z)
{
}

Vector3D::Vector3D(Vector3D vectorStart, Vector3D vectorEnd)
	: m_x(vectorEnd.m_x - vectorStart.m_x)
	, m_y(vectorEnd.m_y - vectorStart.m_y)
	, m_z(vectorEnd.m_z - vectorStart.m_z)
{
}

Vector3D::~Vector3D()
{
}

Vector3D Vector3D::GetInverse()
{
	return Vector3D(-m_x, -m_y, -m_z);
}

Vector3D Vector3D::GetUnitVector() const
{
	DOUBLE inverseMagnitude = 1 / GetMagnitude();
	return Vector3D(m_x * inverseMagnitude,
					m_y * inverseMagnitude,
					m_z * inverseMagnitude);
}

DOUBLE Vector3D::GetMagnitude() const
{
	return sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
}

DOUBLE Vector3D::GetMagnitudeSquared() const
{
	return m_x * m_x + m_y * m_y + m_z * m_z;
}

DOUBLE Vector3D::DotProduct(const Vector3D vector) const
{
	return m_x * vector.m_x + m_y * vector.m_y + m_z * vector.m_z;
}

Vector3D Vector3D::CrossProduct(const Vector3D vector) const
{
	return Vector3D(m_y * vector.m_z - m_z * vector.m_y,
					m_z * vector.m_x - m_x * vector.m_z, 
					m_x * vector.m_y - m_y * vector.m_x);
}

Vector3D Vector3D::Add(const Vector3D vector) const
{
	return Vector3D(m_x + vector.m_x,
					m_y + vector.m_y, 
					m_z + vector.m_z);
}

Vector3D Vector3D::operator+(const Vector3D& vector) const
{
	return Vector3D(m_x + vector.m_x,
					m_y + vector.m_y,
					m_z + vector.m_z);
}

Vector3D Vector3D::Sub(const Vector3D vector) const
{
	return Vector3D(m_x - vector.m_x,
					m_y - vector.m_y,
					m_z - vector.m_z);
}

Vector3D Vector3D::operator-(const Vector3D& vector) const
{
	return Vector3D(m_x - vector.m_x,
					m_y - vector.m_y,
					m_z - vector.m_z);
}

Vector3D Vector3D::ScalarMultiply(DOUBLE scalar) const
{
	return Vector3D(m_x * scalar,
					m_y * scalar, 
					m_z * scalar);
}

Vector3D Vector3D::operator*(DOUBLE scalar) const
{
	return Vector3D(m_x * scalar,
					m_y * scalar,
					m_z * scalar);
}

BOOL Vector3D::IsZero() const
{
	return (m_x == 0) && (m_y == 0) && (m_z == 0);
}

BOOL Vector3D::operator!=(const Vector3D& vector)
{
	return this->Sub(vector).GetMagnitudeSquared() >= 0.000000001;
}

BOOL Vector3D::operator==(const Vector3D& vector)
{
	return this->Sub(vector).GetMagnitudeSquared() < 0.000000001;
}