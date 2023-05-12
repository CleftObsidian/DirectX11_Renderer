#pragma once

#include "PhysicsObject.h"

class MovingObject : public PhysicsObject
{
public:
	MovingObject();
	virtual ~MovingObject();

	virtual void DampenMotion(DOUBLE timePassed);

	Vector3D GetAccel();
	void Roll(DOUBLE mult, DOUBLE timePassed);
	void Pitch(DOUBLE mult, DOUBLE timePassed);
	void Yaw(DOUBLE mult, DOUBLE timePassed);
	void AccelForward(DOUBLE mult, DOUBLE timePassed);
	void AccelDrift(DOUBLE mult, DOUBLE timePassed);
	void AccelLift(DOUBLE mult, DOUBLE timePassed);

protected:
	void findIVals();

protected:
	DOUBLE m_rollI;
	DOUBLE m_pitchI;
	DOUBLE m_yawI;

	DOUBLE m_linearDampFactor;
	DOUBLE m_angularDampFactor;

	DOUBLE m_pitchRate;
	DOUBLE m_rollRate;
	DOUBLE m_yawRate;
	DOUBLE m_forwardThrust;
	DOUBLE m_sideThrust;
	DOUBLE m_verticleThrust;

	Vector3D m_accel;
};