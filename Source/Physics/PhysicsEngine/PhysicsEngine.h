#pragma once

#include "OctreeNode.h"

class PhysicsEngine
{
public:
	PhysicsEngine() = delete;
	PhysicsEngine(DOUBLE timestep);
	~PhysicsEngine();

	BOOL IsBodyInUse(UINT16 bodyID);
	void SetGravity(Vector3D gravity);
	void SetOctree(BOOL useOctree, Vector3D octreeOrigin = Vector3D(0, 0, 0), DOUBLE octreeSize = 1000, DOUBLE octreeMinSize = 20);
	DOUBLE GetTimestep();
	void IterateEngineTimestep();
	void IterateEngine(DOUBLE secondsPassed);
	void AddRigidBody(RigidBody* body);
	BOOL RemoveRigidBody(RigidBody* body);
	void RemoveAllBodies();
	OctreeNode* GetOctreeRoot();

private:
	UINT16 giveID();
	void pushBodiesApart(RigidBody* collider, RigidBody* collidee, const Vector3D nV, DOUBLE colDepth);
	void resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Vector3D colPoint, const std::vector<ConvexHull::ColPointInfo> supPoints, DOUBLE restitution);

	BOOL getColDetectInfo(RigidBody* body1, RigidBody* body2, std::vector<ConvexHull::ColPointInfo>* supPoints, Vector3D* norm,
		Vector3D* colPoint, DOUBLE* colDepthOut, BOOL* isFaceCollision, RigidBody** collider, RigidBody** collidee);

	void detectAndResolveCollisions(RigidBody* body1, RigidBody* body2);

private:
	Vector3D m_gravity;
	DOUBLE m_timestep;
	DOUBLE m_fpsCap;
	DOUBLE m_reductionVel;
	DOUBLE m_staticVel;
	BOOL m_bUseOctree;
	Vector3D m_octreeOrigin;
	DOUBLE m_octreeSize;
	DOUBLE m_octreeMin;
	std::vector<RigidBody*> m_apRigidBodies;
	OctreeNode* m_pRoot;
};