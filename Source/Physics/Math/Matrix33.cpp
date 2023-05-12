#include "Matrix33.h"

Matrix33::Matrix33()
	: m_aaElements{0.0}
{
}

Matrix33::Matrix33(DOUBLE a_values[9])
	: m_aaElements{ {a_values[0], a_values[1], a_values[2]},
				   {a_values[3], a_values[4], a_values[5]},
				   {a_values[6], a_values[7], a_values[8]} }
{
}

Matrix33::~Matrix33()
{
}

Matrix33 Matrix33::Invert() const
{
	Matrix33 out;
	out.m_aaElements[0][0] = m_aaElements[1][1] * m_aaElements[2][2] - m_aaElements[1][2] * m_aaElements[2][1];
	out.m_aaElements[0][1] = m_aaElements[0][2] * m_aaElements[2][1] - m_aaElements[0][1] * m_aaElements[2][2];
	out.m_aaElements[0][2] = m_aaElements[0][1] * m_aaElements[1][2] - m_aaElements[0][2] * m_aaElements[1][1];
	out.m_aaElements[1][0] = m_aaElements[1][2] * m_aaElements[2][0] - m_aaElements[1][0] * m_aaElements[2][2];
	out.m_aaElements[1][1] = m_aaElements[0][0] * m_aaElements[2][2] - m_aaElements[0][2] * m_aaElements[2][0];
	out.m_aaElements[1][2] = m_aaElements[0][2] * m_aaElements[0][3] - m_aaElements[0][0] * m_aaElements[1][2];
	out.m_aaElements[2][0] = m_aaElements[1][0] * m_aaElements[2][1] - m_aaElements[1][1] * m_aaElements[2][0];
	out.m_aaElements[2][1] = m_aaElements[0][1] * m_aaElements[2][0] - m_aaElements[0][0] * m_aaElements[2][1];
	out.m_aaElements[2][2] = m_aaElements[0][0] * m_aaElements[1][1] - m_aaElements[0][1] * m_aaElements[1][0];

	DOUBLE s = 1.0 / (m_aaElements[0][0] * (m_aaElements[1][1] * m_aaElements[2][2] - m_aaElements[1][2] * m_aaElements[2][1])
					- m_aaElements[0][1] * (m_aaElements[1][0] * m_aaElements[2][2] - m_aaElements[1][2] * m_aaElements[2][0])
					+ m_aaElements[0][2] * (m_aaElements[1][0] * m_aaElements[2][1] - m_aaElements[1][1] * m_aaElements[2][0]));

	for (UINT i = 0; i < 9; ++i)
	{
		out.m_aaElements[i / 3][i % 3] *= s;
	}

	return out;
}

Vector3D Matrix33::operator*(const Vector3D& vector) const
{
	return Vector3D(m_aaElements[0][0] * vector.m_x + m_aaElements[0][1] * vector.m_y + m_aaElements[0][2] * vector.m_z,
					m_aaElements[1][0] * vector.m_x + m_aaElements[1][1] * vector.m_y + m_aaElements[1][2] * vector.m_z,
					m_aaElements[2][0] * vector.m_x + m_aaElements[2][1] * vector.m_y + m_aaElements[2][2] * vector.m_z);
}

Matrix33 Matrix33::operator*(const Matrix33& matrix) const
{
	Matrix33 out;
	for (UINT i = 0; i < 3; ++i) {
		for (UINT j = 0; j < 3; j++) {
			for (UINT k = 0; k < 3; k++) {
				out.m_aaElements[i][j] += m_aaElements[i][k] * matrix.m_aaElements[k][j];
			}
		}
	}

	return out;
}

Matrix33 Matrix33::operator*(const DOUBLE scalar) const
{
	Matrix33 out;
	for (UINT i = 0; i < 3; ++i) {
		for (UINT j = 0; j < 3; j++) {
			out.m_aaElements[i][j] = m_aaElements[i][j] * scalar;
		}
	}

	return out;
}

Matrix33 Matrix33::operator+(const Matrix33& matrix) const
{
	Matrix33 out;
	for (UINT i = 0; i < 3; ++i) {
		for (UINT j = 0; j < 3; j++) {
			out.m_aaElements[i][j] = m_aaElements[i][j] + matrix.m_aaElements[i][j];
		}
	}

	return out;
}

Matrix33 Matrix33::Skew(Vector3D vector)
{
	Matrix33 out;
	out.m_aaElements[0][1] = -vector.m_z;
	out.m_aaElements[1][0] = vector.m_z;
	out.m_aaElements[0][2] = vector.m_y;
	out.m_aaElements[2][0] = -vector.m_y;
	out.m_aaElements[1][2] = -vector.m_x;
	out.m_aaElements[2][1] = vector.m_x;

	return out;
}

Vector3D Matrix33::NewtonSolve(Vector3D vector, Vector3D fV, const Matrix33& jacobian)
{
	return vector.Sub(jacobian.Invert() * fV);
}
