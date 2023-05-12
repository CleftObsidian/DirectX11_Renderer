#pragma once

#include "PhysicsEngine/PhysicsEngine.h"

class PhysicsObject
{
public:
	PhysicsObject();
	PhysicsObject(const PhysicsObject& object);
	PhysicsObject(RigidBody* body);
	virtual ~PhysicsObject();

	Vector3D GetDir();
	Vector3D GetPos();
	RigidBody* GetRigidBody();

	virtual void Update(FLOAT fElapsedTime);

protected:
	RigidBody* m_pBody;
};