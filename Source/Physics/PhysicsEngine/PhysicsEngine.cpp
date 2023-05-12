#include "PhysicsEngine.h"
#include <chrono>

PhysicsEngine::PhysicsEngine(DOUBLE timestep)
	: m_gravity(Vector3D(0, 20, 0))	// default
	, m_timestep(timestep)
	, m_fpsCap(1)
	, m_reductionVel(2 * timestep * timestep * m_gravity.GetMagnitudeSquared())
	, m_staticVel(0.1)
	, m_bUseOctree()
	, m_octreeOrigin()
	, m_octreeSize()
	, m_octreeMin()
	, m_apRigidBodies()
	, m_pRoot(nullptr)
{
	SetOctree(TRUE);
}

PhysicsEngine::~PhysicsEngine()
{
	for (RigidBody* body : m_apRigidBodies)
	{
		delete body;
	}
}

BOOL PhysicsEngine::IsBodyInUse(UINT16 bodyID)
{
	for (RigidBody* body : m_apRigidBodies)
	{
		if (body->GetID() == bodyID)
		{
			return TRUE;
		}
	}
	return FALSE;
}

void PhysicsEngine::SetGravity(Vector3D gravity)
{
	m_gravity = gravity;
}

void PhysicsEngine::SetOctree(BOOL useOctree, Vector3D octreeOrigin, DOUBLE octreeSize, DOUBLE octreeMinSize)
{
	m_bUseOctree = useOctree;
	m_octreeOrigin = octreeOrigin.Add(Vector3D(-octreeSize / 2.0, -octreeSize / 2.0, -octreeSize / 2.0));
	m_octreeSize = octreeSize;
	m_octreeMin = octreeMinSize;
}

DOUBLE PhysicsEngine::GetTimestep()
{
	return m_timestep;
}

void PhysicsEngine::IterateEngineTimestep()
{
	auto t1 = std::chrono::system_clock::now();
	Vector3D gravityAcceleration(m_gravity.m_x * m_timestep, m_gravity.m_y * m_timestep, m_gravity.m_z * m_timestep);

	for (RigidBody* body : m_apRigidBodies)
	{
		body->AcclerateLineraly(gravityAcceleration);
		body->MoveInTime(m_timestep);
		body->ClearTestedList();
	}
	auto t2 = std::chrono::system_clock::now();
	DOUBLE octreeTime = 0;

	if (m_bUseOctree)
	{
		if (m_pRoot != nullptr)
		{
			delete m_pRoot;
		}
		auto t3 = std::chrono::system_clock::now();
		m_pRoot = new OctreeNode(m_octreeOrigin, m_octreeSize, m_octreeMin);
		auto t4 = std::chrono::system_clock::now();
		for (RigidBody* body : m_apRigidBodies)
		{
			m_pRoot->AddBody(body);
		}
		auto t5 = std::chrono::system_clock::now();
		m_pRoot->ExpandNode();
		auto t6 = std::chrono::system_clock::now();
		std::vector<OctreeNode*> octreeLeafs;
		m_pRoot->GetCollisionLeafs(&octreeLeafs);
		auto t7 = std::chrono::system_clock::now();

		std::chrono::duration<FLOAT> del = t3 - t2;
		std::chrono::duration<FLOAT> create = t4 - t3;
		std::chrono::duration<FLOAT> push = t5 - t4;
		std::chrono::duration<FLOAT> expand = t6 - t5;
		std::chrono::duration<FLOAT> findleaf = t7 - t6;

		//printf("del: %f, cre: %f, push: %f, expa: %f, leaf: %f\n", del.count(), create.count(), push.count(), expand.count(), findleaf.count());

		for (OctreeNode* leaf : octreeLeafs)
		{
			for (UINT i = 0; i < leaf->GetBodies()->size(); ++i)
			{
				for (UINT j = i + 1; j < leaf->GetBodies()->size(); ++j)
				{
					RigidBody* body1 = leaf->GetBodies()->at(i);
					RigidBody* body2 = leaf->GetBodies()->at(j);

					detectAndResolveCollisions(body1, body2);
				}
			}
		}
	}
	else
	{
		for (UINT i = 0; i < m_apRigidBodies.size(); ++i)
		{
			for (UINT j = i + 1; j < m_apRigidBodies.size(); ++j)
			{
				RigidBody* body1 = m_apRigidBodies.at(i);
				RigidBody* body2 = m_apRigidBodies.at(j);

				detectAndResolveCollisions(body1, body2);
			}
		}
	}

	auto t3 = std::chrono::system_clock::now();

	std::chrono::duration<FLOAT> update = t2 - t1;
	std::chrono::duration<FLOAT> checks = t3 - t2;

	//printf("Update time: %f, Octree time: %f, other: %f\n", update.count(), octreeTime, checks.count() - octreeTime);
}

void PhysicsEngine::IterateEngine(DOUBLE secondsPassed)
{
	static DOUBLE timeBuff = 0;
	timeBuff += secondsPassed;
	DOUBLE maxBuff = std::fmax(0.1, m_timestep);
	timeBuff = std::fmin(maxBuff, timeBuff);

	//timestep = secondsPassed;
	while (timeBuff >= m_timestep)
	{
		timeBuff -= m_timestep;
		IterateEngineTimestep();
	}
}

void PhysicsEngine::AddRigidBody(RigidBody* body)
{
	m_apRigidBodies.push_back(body);
	body->SetID(giveID());
}

BOOL PhysicsEngine::RemoveRigidBody(RigidBody* body)
{
	for (UINT index = 0; index < m_apRigidBodies.size(); ++index)
	{
		if (m_apRigidBodies.at(index) == body)
		{
			m_apRigidBodies.erase(m_apRigidBodies.begin() + index);
			return TRUE;
		}
	}
	return FALSE;
}

void PhysicsEngine::RemoveAllBodies()
{
	m_apRigidBodies.clear();
}

OctreeNode* PhysicsEngine::GetOctreeRoot()
{
	return m_pRoot;
}

UINT16 PhysicsEngine::giveID()
{
	static UINT16 id = 0;
	return id++;
}

void PhysicsEngine::pushBodiesApart(RigidBody* collider, RigidBody* collidee, const Vector3D nV, DOUBLE colDepth)
{
	DOUBLE pushDist = colDepth + 0.1;
	DOUBLE dCldr = pushDist / (1 + collider->GetMass() * collidee->GetInverseMass());
	if (collider->GetIsFixed())
	{
		dCldr = 0;
	}
	DOUBLE dCldee = pushDist - dCldr;
	Vector3D cldeTransform = nV.ScalarMultiply(-dCldee);
	Vector3D cldrTransform = nV.ScalarMultiply(dCldr);
	collider->Translate(cldrTransform);
	collidee->Translate(cldeTransform);
}

void PhysicsEngine::resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Vector3D colPoint, const std::vector<ConvexHull::ColPointInfo> supPoints, DOUBLE restitution)
{
	Vector3D vCldrP0 = collider->GetVelocityOfPoint(colPoint);
	Vector3D vCldeP0 = collidee->GetVelocityOfPoint(colPoint);
	Vector3D velRel = vCldeP0.Sub(vCldrP0);
	Vector3D rClde(collidee->GetCenterOfMass(), colPoint);
	Vector3D rCldr(collider->GetCenterOfMass(), colPoint);
	Vector3D jRotAxisCldr = rCldr.CrossProduct(nV);
	Vector3D jRotAxisClde = rClde.CrossProduct(nV);
	DOUBLE invIClde = collidee->FindInverseInertiaOfAxis(jRotAxisClde);
	DOUBLE invICldr = collider->FindInverseInertiaOfAxis(jRotAxisCldr);
	DOUBLE normVel = nV.DotProduct(velRel);
	Vector3D rCldeXn = rClde.CrossProduct(nV);
	Vector3D rCldrXn = rCldr.CrossProduct(nV);

	DOUBLE numerator = -(restitution + 1) * normVel;
	DOUBLE denomenator = (collider->GetInverseMass() + collidee->GetInverseMass() + rCldrXn.GetMagnitudeSquared() * invICldr + rCldeXn.GetMagnitudeSquared() * invIClde);

	DOUBLE impulseMagnitude = abs(numerator / denomenator);
	Vector3D impulse = nV.ScalarMultiply(impulseMagnitude);

	Vector3D velParralel = nV.ScalarMultiply(normVel);
	Vector3D k = velRel.Sub(velParralel);
	Vector3D frictionImpulse;
	if (!k.IsZero())
	{
		k = k.GetUnitVector();
		jRotAxisCldr = rCldr.CrossProduct(k);
		jRotAxisClde = rClde.CrossProduct(k);
		invIClde = collidee->FindInverseInertiaOfAxis(jRotAxisClde);
		invICldr = collider->FindInverseInertiaOfAxis(jRotAxisCldr);
		numerator = k.DotProduct(velRel);
		Vector3D rCldrXk = rCldr.CrossProduct(k);
		Vector3D rCldeXk = rCldr.CrossProduct(k);
		denomenator = (collider->GetInverseMass() + collidee->GetInverseMass() + rCldrXk.GetMagnitudeSquared() * invICldr + rCldeXk.GetMagnitudeSquared() * invIClde);
		DOUBLE frictionImpMag = abs(numerator / denomenator);
		DOUBLE maxFriction = impulseMagnitude * collidee->GetFriction();
		if (frictionImpMag > maxFriction)
		{
			frictionImpMag = maxFriction;
		}
		frictionImpulse = k.ScalarMultiply(frictionImpMag);
	}

	if (!impulse.IsZero()) {
		collider->ApplyImpulseAtPosition(impulse, colPoint);
		impulse = impulse.ScalarMultiply(-1);
		collidee->ApplyImpulseAtPosition(impulse, colPoint);

		// collision history is meant to be used by outside classes to be able to know what physics
		// stuff has taken place since their last checked
		if (collider->IsTrackingCollHistory())
		{
			collider->AddToColHistory(collidee->GetID(), impulseMagnitude);
		}
		if (collidee->IsTrackingCollHistory())
		{
			collidee->AddToColHistory(collider->GetID(), impulseMagnitude);
		}
	}
	if (!frictionImpulse.IsZero())
	{
		collider->ApplyImpulseAtPosition(frictionImpulse, colPoint);
		frictionImpulse = frictionImpulse.ScalarMultiply(-1);
		collidee->ApplyImpulseAtPosition(frictionImpulse, colPoint);
		DOUBLE angVelAfter = collider->GetAngularVelocity().GetMagnitudeSquared();
	}
}

BOOL PhysicsEngine::getColDetectInfo(RigidBody* body1, RigidBody* body2, std::vector<ConvexHull::ColPointInfo>* supPoints, Vector3D* norm,
	Vector3D* colPoint, DOUBLE* colDepthOut, BOOL* isFaceCollision, RigidBody** collider, RigidBody** collidee)
{
	BOOL colFound = FALSE;
	DOUBLE colDepth = -1;
	for (ConvexHull* hullA : *body1->GetHulls())
	{
		for (ConvexHull* hullB : *body2->GetHulls())
		{
			if (!hullA->IsHullsInCollisionRange(hullB))
			{
				continue;
			}

			INT winningCol = -1; //, 0 is A, 1 B, 2 E

			// collisionInfo if hullA is collidee
			std::vector<ConvexHull::ColPointInfo> supPointsA;
			Vector3D normA;
			Vector3D colPointA;
			DOUBLE colDepthA;
			BOOL separatingAxisA;

			// collisionInfo if hullB is collidee
			std::vector<ConvexHull::ColPointInfo> supPointsB;
			Vector3D normB;
			Vector3D colPointB;
			DOUBLE colDepthB;
			BOOL separatingAxisB;

			// collisionInfo in case of edge on edge collision
			Vector3D normE;
			Vector3D colPointE;
			DOUBLE colDepthE;
			BOOL separatingAxisE;

			RigidBody* potCollider = body2;
			RigidBody* potCollidee = body1;

			BOOL aCol = hullA->SATColliderDetect(hullB, &supPointsA, &colPointA, &normA, &colDepthA, &separatingAxisA);
			BOOL bCol = hullB->SATColliderDetect(hullA, &supPointsB, &colPointB, &normB, &colDepthB, &separatingAxisB);
			BOOL eCol = hullA->SATEdgeCol(hullB, &colPointE, &normE, &colDepthE, &separatingAxisE);

			if ((aCol || bCol || eCol) && !(separatingAxisA || separatingAxisB || separatingAxisE))
			{
				DOUBLE winningDepth = 0;

				if (aCol && (!bCol || (bCol && colDepthB > colDepthA)))
				{
					winningCol = 0;
					winningDepth = colDepthA;
					if (eCol && colDepthE < colDepthA)
					{
						winningCol = 2;
						winningDepth = colDepthE;
					}
				}
				else if (bCol)
				{
					winningCol = 1;
					winningDepth = colDepthB;
					if (eCol && colDepthE < colDepthB)
					{
						winningCol = 2;
						winningDepth = colDepthE;
					}
				}
				else
				{
					winningCol = 2;
					winningDepth = colDepthE;
				}

				if (colDepth == -1 || winningDepth > colDepth)
				{
					colFound = TRUE;

					switch (winningCol)
					{
					case 0:
						*collider = body2;
						*collidee = body1;
						*supPoints = supPointsA;
						*norm = normA;
						*colPoint = colPointA;
						colDepth = colDepthA;
						*isFaceCollision = TRUE;
						break;
					case 1:
						*collider = body1;
						*collidee = body2;
						*supPoints = supPointsB;
						*norm = normB;
						*colPoint = colPointB;
						colDepth = colDepthB;
						*isFaceCollision = TRUE;
						break;
					case 2:
						*norm = normE;
						*colPoint = colPointE;
						colDepth = colDepthE;
						*isFaceCollision = FALSE;
						break;
					}
				}
			}
		}
	}
	*colDepthOut = colDepth;
	return colFound;
}

void PhysicsEngine::detectAndResolveCollisions(RigidBody* body1, RigidBody* body2)
{
	static DOUBLE narrowTests = 0;
	static DOUBLE satTests = 0;
	static DOUBLE collApply = 0;

	// checks to avoid unnecessary/unwanted physics
	if (body1->GetIsFixed() && body2->GetIsFixed())
	{
		return;
	}
	if (body1->IsOnNoColList(body2->GetID()) || body2->IsOnNoColList(body1->GetID()))
	{
		return;
	}

	if (body1->IsAlreadyTestedAgainst(body2->GetID()))
	{
		return;
	}
	else
	{
		body1->AddTestedAgainst(body2->GetID());
		body2->AddTestedAgainst(body1->GetID());
	}

	auto t1 = std::chrono::system_clock::now();
	if (!body1->IsBodiesInCollisionRange(body2))
	{
		auto t2 = std::chrono::system_clock::now();
		std::chrono::duration<FLOAT> t = t2 - t1;
		narrowTests += t.count();
		return;
	}
	auto t2 = std::chrono::system_clock::now();

	std::vector<ConvexHull::ColPointInfo> supPoints;
	Vector3D norm;
	Vector3D colPoint;
	DOUBLE colDepth;
	BOOL isFaceCollision = TRUE;

	RigidBody* collider = body2;
	RigidBody* collidee = body1;

	//find the intersection between hulls of each body with greatest penetration,
	//and resolve that collision
	BOOL colFound = getColDetectInfo(body1, body2, &supPoints, &norm, &colPoint, &colDepth, &isFaceCollision, &collider, &collidee);
	DOUBLE ogDepth = colDepth;
	if (colFound && FALSE)
	{
		DOUBLE netT = 0;
		INT maxItr = 5;
		DOUBLE eps = 1;
		DOUBLE t = -m_timestep / 2.0;
		for (UINT i = 0; i < maxItr && colDepth > eps; ++i)
		{
			body1->MoveInTime(t);
			body2->MoveInTime(t);
			netT += t;
			if (getColDetectInfo(body1, body2, &supPoints, &norm, &colPoint, &colDepth, &isFaceCollision, &collider, &collidee))
			{
				t = -abs(t) / 2.0;
			}
			else
			{
				t = abs(t) / 2.0;
			}
		}
		body1->MoveInTime(-netT);
		body2->MoveInTime(-netT);
	}

	auto t3 = std::chrono::system_clock::now();
	if (colFound)
	{
		BOOL contactColl = FALSE;

		if (isFaceCollision)
		{
			//using an average of the points collided is a rough solution to poor contact behavior,
			//has some issues like not picking up torque from friction as well
			contactColl = TRUE;
			Vector3D average(0, 0, 0);
			DOUBLE depth = 0;
			/*for (ConvexHull::ColPointInfo supPoint : supPoints) {
				average.m_x += supPoint.m_point.m_x;
				average.m_y += supPoint.m_point.m_y;
				average.m_z += supPoint.m_point.m_z;
				depth += supPoint.m_penDepth;
			}
			average.m_x /= supPoints.size();
			average.m_y /= supPoints.size();
			average.m_z /= supPoints.size();
			depth /= supPoints.size();*/
			average = colPoint;

			if (collider->IsVerifiedCollisionPointNotExiting(collidee, norm, average))
			{
				Vector3D vCldrP0 = collider->GetVelocityOfPoint(colPoint);
				Vector3D vCldeP0 = collidee->GetVelocityOfPoint(colPoint);
				Vector3D velRel = vCldeP0.Sub(vCldrP0);
				double normVel = norm.DotProduct(velRel);
				resolveImpulses(collider, collidee, norm, average, supPoints, (normVel < m_gravity.GetMagnitude() / (2.25)) ? 0 : collidee->GetRestitution());
			}
		}
		else if (!collidee->IsVerifiedCollisionPointNotExiting(collider, norm, colPoint))
		{
			resolveImpulses(collider, collidee, norm, colPoint, supPoints, (collidee->GetRestitution() + collider->GetRestitution()) / 2.0);
		}

		pushBodiesApart(collider, collidee, norm, (contactColl) ? 1/*0.15*/ * colDepth : colDepth);
		auto t4 = std::chrono::system_clock::now();

		std::chrono::duration<float> narrow = t2 - t1;
		std::chrono::duration<float> sat = t3 - t2;
		std::chrono::duration<float> resolve = t4 - t3;

		narrowTests += narrow.count();
		satTests += sat.count();
		collApply += resolve.count();

		//printf("narrowTest: %f, satTest: %f, collApply: %f\n", narrowTests, satTests, collApply);
	}
}
