#include "PhysicsObject.h"

PhysicsObject::PhysicsObject()
	: m_pBody(nullptr)
{
}

PhysicsObject::PhysicsObject(const PhysicsObject& object)
	: m_pBody(new RigidBody(*object.m_pBody))
{
}

PhysicsObject::PhysicsObject(RigidBody* body)
	: m_pBody(body)
{
}

PhysicsObject::~PhysicsObject()
{
	delete m_pBody;
}

Vector3D PhysicsObject::GetDir()
{
	return m_pBody->GetOrientation().Rotate(Vector3D(1, 0, 0));
}

Vector3D PhysicsObject::GetPos()
{
	return m_pBody->GetCenterOfMass();
}

RigidBody* PhysicsObject::GetRigidBody()
{
	return m_pBody;
}

void PhysicsObject::Update(FLOAT fElapsedTime)
{
	UNREFERENCED_PARAMETER(fElapsedTime);
}