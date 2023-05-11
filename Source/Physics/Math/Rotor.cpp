#include "Rotor.h"

Rotor::Rotor()
	: m_a(1.0)
	, m_b(0.0)
	, m_c(0.0)
	, m_d(0.0)
{
}

Rotor::Rotor(DOUBLE a, DOUBLE b, DOUBLE c, DOUBLE d)
	: m_a(a)
	, m_b(b)
	, m_c(c)
	, m_d(d)
{
}

Rotor::Rotor(Vector3D unitAxis, DOUBLE theta)
	: m_a(cos(theta / 2.0))
	, m_b(-sin(theta / 2.0) * unitAxis.m_z) // ele2
	, m_c(-sin(theta / 2.0) * unitAxis.m_x) // ele3
	, m_d(-sin(theta / 2.0) * unitAxis.m_y) // ele1
{
}

Rotor::~Rotor()
{
}

Vector3D Rotor::Rotate(Vector3D p)
{
	/* RpR^-1									:: p-vector
		= (a + B)p(a - B)						:: a-scalar, B-Bivector
		= (ap + Bp)(a - B)
		= (ap + u + t)(a - B)					:: u-vector output Bp, t-trivector output Bp
		= (aap + au + at - apB + uB + tB)
		= (aap + au + at - a(-u + t) - uB - tB)
		= (aap + 2au - uB - tB)
		= (aap + 2au - k - w)					:: k-vector output uB (no trivector output), w-vector output tB
	*/

	Vector3D u(m_b * p.m_y - m_d * p.m_z,
			   m_c * p.m_z - m_b * p.m_x,
			   m_d * p.m_x - m_c * p.m_y);

	Vector3D k(-m_b * u.m_y + m_d * u.m_z,
			   -m_c * u.m_z + m_b * u.m_x,
			   -m_d * u.m_x + m_c * u.m_y);

	Vector3D w = Vector3D(-m_c, -m_d, -m_b).ScalarMultiply(m_b * p.m_z + m_c * p.m_x + m_d * p.m_y);

	return p.ScalarMultiply(m_a * m_a).Add(u.ScalarMultiply(2.0 * m_a)).Sub(k).Sub(w);
}

Rotor Rotor::ApplyRotor(Rotor rotor)
{
	return Rotor(m_a * rotor.m_a - m_b * rotor.m_b - m_c * rotor.m_c - m_d * rotor.m_d,
				 m_a * rotor.m_b + rotor.m_a * m_b + (rotor.m_d * m_c - rotor.m_c * m_d),
				 m_a * rotor.m_c + rotor.m_a * m_c + (rotor.m_b * m_d - rotor.m_d * m_b),
				 m_a * rotor.m_d + rotor.m_a * m_d + (rotor.m_c * m_b - rotor.m_b * m_c));
}

Rotor Rotor::GetInverse()
{
	return Rotor(m_a, -m_b, -m_c, -m_d);
}

BOOL Rotor::operator!=(const Rotor& rotor)
{
	return (m_a != rotor.m_a) || (m_b != rotor.m_b) || (m_c != rotor.m_c) || (m_d != rotor.m_d);
}

BOOL Rotor::operator==(const Rotor& rotor)
{
	return m_a == rotor.m_a && m_b == rotor.m_b && m_c == rotor.m_c && m_d == rotor.m_d;
}
