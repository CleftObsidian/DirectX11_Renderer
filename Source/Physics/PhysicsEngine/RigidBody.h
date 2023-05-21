#pragma once

#include "CollisionInfo.h"
#include "ConvexHull.h"
#include "Math/Rotor.h"

class RigidBody
{
public:
	RigidBody();
	RigidBody(const std::vector<std::shared_ptr<ConvexHull>> hulls, DOUBLE density, DOUBLE friction, DOUBLE resistution, BOOL isFixed, BOOL custumCOM = FALSE, Vector3D com = Vector3D(0, 0, 0));
	RigidBody(const RigidBody& body);
	~RigidBody();

	Vector3D FindVectorRelativeToBodyFrame(const Vector3D vector);
	std::vector<Vector3D>* GetAllPoints();
	BOOL IsBodiesInCollisionRange(RigidBody* body);
	DOUBLE GetCollisionRadius() const;
	DOUBLE GetCollisionRadiusSquared() const;
	Vector3D GetAngularVelocity();
	std::vector<std::shared_ptr<ConvexHull>>* GetHulls();
	Vector3D GetCenterOfMass();
	Vector3D GetVelocityOfPointDueToAngularVelocity(const Vector3D point) const;
	Vector3D GetVelocityOfPoint(const Vector3D point) const;
	Vector3D GetVelocity();
	BOOL IsVerifiedCollisionPointNotExiting(RigidBody* body, const Vector3D normalVector, const Vector3D point);
	void ApplyImpulseAtPosition(const Vector3D impulse, const Vector3D position);
	DOUBLE FindInverseInertiaOfAxis(const Vector3D inputAxis);
	DOUBLE GetRadialDistanceOfPoint(const Vector3D point);
	DOUBLE GetMass() const;
	DOUBLE GetInverseMass() const;
	DOUBLE GetFriction() const;
	DOUBLE GetRestitution() const;
	UINT16 GetID();
	void AddNoCol(UINT16 ID);
	BOOL IsOnNoColList(UINT16 ID);
	void SetToOrientation(Rotor orientation);
	void SetVelocity(Vector3D velocity);
	void SetAngularVelocity(Vector3D angularVelocity);
	void SetID(UINT16 ID);
	void Translate(const Vector3D translation);
	void AcclerateLineraly(const Vector3D changeInVelocity);
	void MoveInTime(DOUBLE time);
	BOOL IsAlreadyTestedAgainst(UINT16 colliderID);
	void AddTestedAgainst(UINT16 colliderID);
	void ClearTestedList();
	void AddToColHistory(UINT16 collider, DOUBLE magnitude);
	std::vector<CollisionInfo>* GetCollHistory();
	void ClearCollHistory();
	BOOL IsTrackingCollHistory();
	void SetTrackCollHistory(BOOL b);
	BOOL GetIsFixed();
	Rotor GetOrientation();
	Vector3D GetDimensions();
	Vector3D GetDimCenter();

private:
	void copyPoints();
	void findBodyMassAndInertia(DOUBLE density, BOOL useCustumCOM = FALSE, Vector3D custumCOM = Vector3D(0, 0, 0));
	DOUBLE getInertiaOfAxis(const Vector3D axisIn);
	void findCollisionRadius();
	void findDimensions();
	Vector3D gyroAccel(DOUBLE time);

private:
	std::vector<Vector3D*> m_apPointsToTransform;
	std::vector<Vector3D> m_aPointsOG;
	std::vector<std::shared_ptr<ConvexHull>> m_apHulls;
	std::vector<UINT16> m_aTestedList;
	std::vector<UINT16> m_aNoColList;
	std::vector<CollisionInfo> m_aCollHistory;
	Vector3D m_centerOfMass;
	Vector3D m_velocity;
	Vector3D m_angularVelocity;
	Rotor m_orientation;

	Vector3D m_fixedDimensions;
	Vector3D m_dimCenter;

	DOUBLE m_collisionRadius;
	DOUBLE m_collisionRadiusSquared;
	DOUBLE m_mass;
	DOUBLE m_inverseMass;
	DOUBLE m_friction;
	DOUBLE m_restitution;
	Matrix33 m_inertiaTensor;
	Matrix33 m_tensorInverse;
	BOOL m_bIsFixed;
	BOOL m_bIsTrackHistory;
	UINT16 m_ID;
};