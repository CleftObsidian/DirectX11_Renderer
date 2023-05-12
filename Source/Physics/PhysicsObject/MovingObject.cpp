#include "MovingObject.h"

MovingObject::MovingObject()
	: PhysicsObject()
	, m_rollI()
	, m_pitchI()
	, m_yawI()
	, m_linearDampFactor()
	, m_angularDampFactor()
	, m_pitchRate()
	, m_rollRate()
	, m_yawRate()
	, m_forwardThrust()
	, m_sideThrust()
	, m_verticleThrust()
	, m_accel()
{
}

MovingObject::~MovingObject()
{
}

void MovingObject::DampenMotion(DOUBLE timePassed)
{
	FLOAT t = fmin(1, timePassed);
	Vector3D vel = m_pBody->GetVelocity();
	Vector3D aVel = m_pBody->GetAngularVelocity();

	m_pBody->SetVelocity(vel.Sub(vel.ScalarMultiply(t * m_linearDampFactor)));
	m_pBody->SetAngularVelocity(aVel.Sub(aVel.ScalarMultiply(t * m_angularDampFactor)));
}

Vector3D MovingObject::GetAccel()
{
	return m_accel;
}

void MovingObject::Roll(DOUBLE mult, DOUBLE timePassed)
{
	Vector3D axis = m_pBody->GetOrientation().Rotate(Vector3D(0, 1, 0));
	Vector3D impulse = m_pBody->GetOrientation().Rotate(Vector3D(0, 0, mult * m_rollRate * timePassed * m_yawI / 2.0));
	m_pBody->ApplyImpulseAtPosition(impulse, m_pBody->GetCenterOfMass().Add(axis));
	m_pBody->ApplyImpulseAtPosition(impulse.GetInverse(), m_pBody->GetCenterOfMass().Sub(axis));
}

void MovingObject::Pitch(DOUBLE mult, DOUBLE timePassed)
{
	Vector3D axis = m_pBody->GetOrientation().Rotate(Vector3D(0, 1, 0));
	Vector3D impulse = m_pBody->GetOrientation().Rotate(Vector3D(mult * m_pitchRate * timePassed * m_pitchI / 2.0, 0, 0));
	m_pBody->ApplyImpulseAtPosition(impulse, m_pBody->GetCenterOfMass().Add(axis));
	m_pBody->ApplyImpulseAtPosition(impulse.GetInverse(), m_pBody->GetCenterOfMass().Sub(axis));
}

void MovingObject::Yaw(DOUBLE mult, DOUBLE timePassed)
{
	Vector3D axis = m_pBody->GetOrientation().Rotate(Vector3D(0, 1, 0));
	Vector3D impulse = m_pBody->GetOrientation().Rotate(Vector3D(mult * m_yawRate * timePassed * m_yawI / 2.0, 0, 0));
	m_pBody->ApplyImpulseAtPosition(impulse, m_pBody->GetCenterOfMass().Add(axis));
	m_pBody->ApplyImpulseAtPosition(impulse.GetInverse(), m_pBody->GetCenterOfMass().Sub(axis));
}

void MovingObject::AccelForward(DOUBLE mult, DOUBLE timePassed)
{
	Vector3D axis = m_pBody->GetOrientation().Rotate(Vector3D(1, 0, 0));
	double impulse = mult * m_forwardThrust * timePassed * m_pBody->GetMass();
	m_pBody->ApplyImpulseAtPosition(axis.ScalarMultiply(impulse), m_pBody->GetCenterOfMass());
}

void MovingObject::AccelDrift(DOUBLE mult, DOUBLE timePassed)
{
	Vector3D axis = m_pBody->GetOrientation().Rotate(Vector3D(0, 0, -1));
	double impulse = mult * m_sideThrust * timePassed * m_pBody->GetMass();
	m_pBody->ApplyImpulseAtPosition(axis.ScalarMultiply(impulse), m_pBody->GetCenterOfMass());
}

void MovingObject::AccelLift(DOUBLE mult, DOUBLE timePassed)
{
	Vector3D axis = m_pBody->GetOrientation().Rotate(Vector3D(1, -1, 0));
	double impulse = mult * m_verticleThrust * timePassed * m_pBody->GetMass();
	m_pBody->ApplyImpulseAtPosition(axis.ScalarMultiply(impulse), m_pBody->GetCenterOfMass());
}

void MovingObject::findIVals()
{
	m_rollI = 1.0 / m_pBody->FindInverseInertiaOfAxis(Vector3D(1, 0, 0));
	m_pitchI = 1.0 / m_pBody->FindInverseInertiaOfAxis(Vector3D(0, 0, 1));
	m_yawI = 1.0 / m_pBody->FindInverseInertiaOfAxis(Vector3D(0, 1, 0));
}
