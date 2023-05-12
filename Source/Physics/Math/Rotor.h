#pragma once

#include "Vector3D.h"

class Rotor final
{
public:
	Rotor();
	Rotor(DOUBLE a, DOUBLE b, DOUBLE c, DOUBLE d);
	Rotor(Vector3D unitAxis, DOUBLE theta);
	~Rotor();

	Vector3D Rotate(Vector3D p);
	Rotor ApplyRotor(Rotor rotor);
	Rotor GetInverse();

	BOOL operator!=(const Rotor& rotor);
	BOOL operator==(const Rotor& rotor);

private:
	DOUBLE m_a;
	DOUBLE m_b;
	DOUBLE m_c;
	DOUBLE m_d;
};