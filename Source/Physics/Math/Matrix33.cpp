#include "Matrix33.h"

Matrix33::Matrix33()
	: aa_elements{0.0}
{
}

Matrix33::Matrix33(DOUBLE a_values[9])
	: aa_elements{ {a_values[0], a_values[1], a_values[2]},
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
	out.aa_elements[0][0] = aa_elements[1][1] * aa_elements[2][2] - aa_elements[1][2] * aa_elements[2][1];
	out.aa_elements[0][1] = aa_elements[0][2] * aa_elements[2][1] - aa_elements[0][1] * aa_elements[2][2];
	out.aa_elements[0][2] = aa_elements[0][1] * aa_elements[1][2] - aa_elements[0][2] * aa_elements[1][1];
	out.aa_elements[1][0] = aa_elements[1][2] * aa_elements[2][0] - aa_elements[1][0] * aa_elements[2][2];
	out.aa_elements[1][1] = aa_elements[0][0] * aa_elements[2][2] - aa_elements[0][2] * aa_elements[2][0];
	out.aa_elements[1][2] = aa_elements[0][2] * aa_elements[0][3] - aa_elements[0][0] * aa_elements[1][2];
	out.aa_elements[2][0] = aa_elements[1][0] * aa_elements[2][1] - aa_elements[1][1] * aa_elements[2][0];
	out.aa_elements[2][1] = aa_elements[0][1] * aa_elements[2][0] - aa_elements[0][0] * aa_elements[2][1];
	out.aa_elements[2][2] = aa_elements[0][0] * aa_elements[1][1] - aa_elements[0][1] * aa_elements[1][0];

	DOUBLE s = 1.0 / (aa_elements[0][0] * (aa_elements[1][1] * aa_elements[2][2] - aa_elements[1][2] * aa_elements[2][1])
					- aa_elements[0][1] * (aa_elements[1][0] * aa_elements[2][2] - aa_elements[1][2] * aa_elements[2][0])
					+ aa_elements[0][2] * (aa_elements[1][0] * aa_elements[2][1] - aa_elements[1][1] * aa_elements[2][0]));

	for (int i = 0; i < 9; ++i)
	{
		out.aa_elements[i / 3][i % 3] *= s;
	}

	return out;
}

Vector3D Matrix33::operator*(const Vector3D& vector) const
{
	return Vector3D(aa_elements[0][0] * vector.m_x + aa_elements[0][1] * vector.m_y + aa_elements[0][2] * vector.m_z,
					aa_elements[1][0] * vector.m_x + aa_elements[1][1] * vector.m_y + aa_elements[1][2] * vector.m_z,
					aa_elements[2][0] * vector.m_x + aa_elements[2][1] * vector.m_y + aa_elements[2][2] * vector.m_z);
}

Matrix33 Matrix33::operator*(const Matrix33& matrix) const
{
	Matrix33 out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				out.aa_elements[i][j] += aa_elements[i][k] * matrix.aa_elements[k][j];
			}
		}
	}

	return out;
}

Matrix33 Matrix33::operator*(const DOUBLE scalar) const
{
	Matrix33 out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.aa_elements[i][j] = aa_elements[i][j] * scalar;
		}
	}

	return out;
}

Matrix33 Matrix33::operator+(const Matrix33& matrix) const
{
	Matrix33 out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.aa_elements[i][j] = aa_elements[i][j] + matrix.aa_elements[i][j];
		}
	}

	return out;
}

Matrix33 Matrix33::SKew(Vector3D vector)
{
	Matrix33 out;
	out.aa_elements[0][1] = -vector.m_z;
	out.aa_elements[1][0] = vector.m_z;
	out.aa_elements[0][2] = vector.m_y;
	out.aa_elements[2][0] = -vector.m_y;
	out.aa_elements[1][2] = -vector.m_x;
	out.aa_elements[2][1] = vector.m_x;

	return out;
}

Vector3D Matrix33::NewtonSolve(Vector3D vector, Vector3D fV, const Matrix33& jacobian)
{
	return vector.Sub(jacobian.Invert() * fV);
}
