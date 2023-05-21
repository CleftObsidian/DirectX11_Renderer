#include "RigidBody.h"
#include "Math/Transformation3D.h"
#include <stdio.h>

RigidBody::RigidBody()
	: m_apPointsToTransform()
	, m_aPointsOG()
	, m_apHulls()
	, m_aTestedList()
	, m_aNoColList()
	, m_aCollHistory()
	, m_centerOfMass()
	, m_velocity()
	, m_angularVelocity()
	, m_orientation()
	, m_fixedDimensions()
	, m_dimCenter()
	, m_collisionRadius()
	, m_collisionRadiusSquared()
	, m_mass()
	, m_inverseMass()
	, m_friction()
	, m_restitution()
	, m_inertiaTensor()
	, m_tensorInverse()
	, m_bIsFixed()
	, m_bIsTrackHistory()
	, m_ID()
{
}

RigidBody::RigidBody(const std::vector<std::shared_ptr<ConvexHull>> hulls, DOUBLE density, DOUBLE friction, DOUBLE resistution, BOOL isFixed, BOOL custumCOM, Vector3D com)
	: m_apPointsToTransform()
	, m_aPointsOG()
	, m_apHulls(hulls)
	, m_aTestedList()
	, m_aNoColList()
	, m_aCollHistory()
	, m_centerOfMass()
	, m_velocity()
	, m_angularVelocity()
	, m_orientation()
	, m_fixedDimensions()
	, m_dimCenter()
	, m_collisionRadius()
	, m_collisionRadiusSquared()
	, m_mass()
	, m_inverseMass()
	, m_friction(friction)
	, m_restitution(resistution)
	, m_inertiaTensor()
	, m_tensorInverse()
	, m_bIsFixed(isFixed)
	, m_bIsTrackHistory(FALSE)
	, m_ID(-1)	// unitialized
{
	findBodyMassAndInertia(density, custumCOM, com);

	for (std::shared_ptr<ConvexHull> hull : hulls)
	{
		for (RigidSurface* surface : *hull->GetSurfaces())
		{
			for (Vector3D* point : *surface->GetPoints())
			{
				m_apPointsToTransform.push_back(point);
			}
		}
		m_apPointsToTransform.push_back(hull->GetCOMPointer());
	}

	if (m_bIsFixed)
	{
		findDimensions();
	}

	findCollisionRadius();

	copyPoints();
}

RigidBody::RigidBody(const RigidBody& body)
	: m_apPointsToTransform()
	, m_aPointsOG()
	, m_apHulls()
	, m_aTestedList()
	, m_aNoColList()
	, m_aCollHistory()
	, m_centerOfMass(body.m_centerOfMass)
	, m_velocity()
	, m_angularVelocity()
	, m_orientation()
	, m_fixedDimensions()
	, m_dimCenter()
	, m_collisionRadius()
	, m_collisionRadiusSquared()
	, m_mass(body.m_mass)
	, m_inverseMass(body.m_inverseMass)
	, m_friction(body.m_friction)
	, m_restitution(body.m_restitution)
	, m_inertiaTensor(body.m_inertiaTensor)
	, m_tensorInverse()
	, m_bIsFixed(body.m_bIsFixed)
	, m_bIsTrackHistory(FALSE)
	, m_ID(-1)	// unitialized
{
	for (std::shared_ptr<ConvexHull> hull : body.m_apHulls)
	{
		m_apHulls.push_back(std::make_shared<ConvexHull>(*hull));
	}

	for (std::shared_ptr<ConvexHull> hull : m_apHulls)
	{
		for (RigidSurface* s : *hull->GetSurfaces())
		{
			for (Vector3D* p : *s->GetPoints())
			{
				m_apPointsToTransform.push_back(p);
			}
		}
		m_apPointsToTransform.push_back(hull->GetCOMPointer());
	}

	if (m_bIsFixed)
	{
		findDimensions();
	}

	findCollisionRadius();

	copyPoints();
}

RigidBody::~RigidBody()
{
	/*for (std::shared_ptr<ConvexHull> hull : m_apHulls)
	{
		delete hull;
	}*/
}

Vector3D RigidBody::FindVectorRelativeToBodyFrame(const Vector3D vector)
{
	return m_orientation.GetInverse().Rotate(vector);
}

std::vector<Vector3D>* RigidBody::GetAllPoints()
{
	return &m_aPointsOG;
}

BOOL RigidBody::IsBodiesInCollisionRange(RigidBody* body)
{
	DOUBLE xDist = m_centerOfMass.m_x - body->GetCenterOfMass().m_x;
	DOUBLE yDist = m_centerOfMass.m_y - body->GetCenterOfMass().m_y;
	DOUBLE zDist = m_centerOfMass.m_z - body->GetCenterOfMass().m_z;

	DOUBLE sqrDist = xDist * xDist + yDist * yDist + zDist * zDist;
	DOUBLE colDist = (m_collisionRadius + body->GetCollisionRadius()) * (m_collisionRadius + body->GetCollisionRadius());

	return sqrDist <= colDist;
}

DOUBLE RigidBody::GetCollisionRadius() const
{
	return m_collisionRadius;
}

DOUBLE RigidBody::GetCollisionRadiusSquared() const
{
	return m_collisionRadiusSquared;
}

Vector3D RigidBody::GetAngularVelocity()
{
	return m_angularVelocity;
}

std::vector<std::shared_ptr<ConvexHull>>* RigidBody::GetHulls()
{
	return &m_apHulls;
}

Vector3D RigidBody::GetCenterOfMass()
{
	return m_centerOfMass;
}

Vector3D RigidBody::GetVelocityOfPointDueToAngularVelocity(const Vector3D point) const
{
	return m_angularVelocity.CrossProduct(Vector3D(m_centerOfMass, point));
}

Vector3D RigidBody::GetVelocityOfPoint(const Vector3D point) const
{
	return GetVelocityOfPointDueToAngularVelocity(point).Add(m_velocity);
}

Vector3D RigidBody::GetVelocity()
{
	return m_velocity;
}

BOOL RigidBody::IsVerifiedCollisionPointNotExiting(RigidBody* body, const Vector3D normalVector, const Vector3D point)
{
	Vector3D vPThisBody = GetVelocityOfPoint(point);
	Vector3D vPOtherBody = body->GetVelocityOfPoint(point);
	Vector3D vPRelative = vPThisBody.Sub(vPOtherBody);

	return vPRelative.DotProduct(normalVector) < 0;
}

void RigidBody::ApplyImpulseAtPosition(const Vector3D impulse, const Vector3D position)
{
	if (m_bIsFixed)
	{
		return;
	}
	m_velocity.m_x += impulse.m_x * m_inverseMass;
	m_velocity.m_y += impulse.m_y * m_inverseMass;
	m_velocity.m_z += impulse.m_z * m_inverseMass;

	Vector3D comToPoint(m_centerOfMass, position);
	Vector3D deltaW = comToPoint.CrossProduct(impulse);
	deltaW = deltaW.ScalarMultiply(FindInverseInertiaOfAxis(deltaW));
	m_angularVelocity = m_angularVelocity.Add(deltaW);
}

DOUBLE RigidBody::FindInverseInertiaOfAxis(const Vector3D inputAxis)
{
	if (m_bIsFixed || inputAxis.IsZero())
	{
		return 0;
	}

	return 1.0 / getInertiaOfAxis(inputAxis);
}

DOUBLE RigidBody::GetRadialDistanceOfPoint(const Vector3D point)
{
	DOUBLE dx = m_centerOfMass.m_x - point.m_x;
	DOUBLE dy = m_centerOfMass.m_y - point.m_y;
	DOUBLE dz = m_centerOfMass.m_z - point.m_z;

	return dx * dx + dy * dy + dz * dz;
}

DOUBLE RigidBody::GetMass() const
{
	return m_mass;
}

DOUBLE RigidBody::GetInverseMass() const
{
	return m_inverseMass;
}

DOUBLE RigidBody::GetFriction() const
{
	return m_friction;
}

DOUBLE RigidBody::GetRestitution() const
{
	return m_restitution;
}

UINT16 RigidBody::GetID()
{
	return m_ID;
}

void RigidBody::AddNoCol(UINT16 ID)
{
	m_aNoColList.push_back(ID);
}

BOOL RigidBody::IsOnNoColList(UINT16 ID)
{
	for (UINT16 id : m_aNoColList)
	{
		if (id == ID)
		{
			return TRUE;
		}
	}
	return FALSE;
}

void RigidBody::SetToOrientation(Rotor orientation)
{
	orientation = orientation;
	for (UINT pointIndex = 0; pointIndex < m_aPointsOG.size(); ++pointIndex)
	{
		*m_apPointsToTransform.at(pointIndex) = m_centerOfMass.Add(orientation.Rotate(m_aPointsOG.at(pointIndex)));
	}
	if (m_bIsFixed)
	{
		findDimensions();
	}
}

void RigidBody::SetVelocity(Vector3D velocity)
{
	m_velocity = velocity;
}

void RigidBody::SetAngularVelocity(Vector3D angularVelocity)
{
	m_angularVelocity = angularVelocity;
}

void RigidBody::SetID(UINT16 ID)
{
	m_ID = ID;
}

void RigidBody::Translate(const Vector3D translation)
{
	m_centerOfMass = m_centerOfMass.Add(translation);
	Transformation3D::TranslatePoints(&m_apPointsToTransform, translation);
}

void RigidBody::AcclerateLineraly(const Vector3D changeInVelocity)
{
	if (m_bIsFixed)
	{
		return;
	}
	m_velocity = m_velocity.Add(changeInVelocity);
}

void RigidBody::MoveInTime(DOUBLE time)
{
	Vector3D linMove = m_velocity.ScalarMultiply(time);
	m_centerOfMass = m_centerOfMass.Add(linMove);

	if (!m_angularVelocity.IsZero())
	{
		m_angularVelocity = gyroAccel(time);

		DOUBLE rotationMagnitude = m_angularVelocity.GetMagnitude() * time;
		Vector3D rotationAxis = m_angularVelocity.GetUnitVector();

		SetToOrientation(m_orientation.ApplyRotor(Rotor(rotationAxis, rotationMagnitude)));
	}
	else
	{
		Transformation3D::TranslatePoints(&m_apPointsToTransform, linMove);
	}
}

BOOL RigidBody::IsAlreadyTestedAgainst(UINT16 colliderID)
{
	for (UINT16 id : m_aTestedList)
	{
		if (id == colliderID)
		{
			return TRUE;
		}
	}
	return FALSE;
}

void RigidBody::AddTestedAgainst(UINT16 colliderID)
{
	m_aTestedList.push_back(colliderID);
}

void RigidBody::ClearTestedList()
{
	m_aTestedList.clear();
}

void RigidBody::AddToColHistory(UINT16 collider, DOUBLE magnitude)
{
	m_aCollHistory.push_back(CollisionInfo(collider, magnitude));
}

std::vector<CollisionInfo>* RigidBody::GetCollHistory()
{
	return &m_aCollHistory;
}

void RigidBody::ClearCollHistory()
{
	m_aCollHistory.clear();
}

BOOL RigidBody::IsTrackingCollHistory()
{
	return m_bIsTrackHistory;
}

void RigidBody::SetTrackCollHistory(BOOL b)
{
	m_bIsTrackHistory = b;
}

BOOL RigidBody::GetIsFixed()
{
	return m_bIsFixed;
}

Rotor RigidBody::GetOrientation()
{
	return m_orientation;
}

Vector3D RigidBody::GetDimensions()
{
	return m_fixedDimensions;
}

Vector3D RigidBody::GetDimCenter()
{
	return m_dimCenter;
}

void RigidBody::copyPoints()
{
	for (Vector3D* point : m_apPointsToTransform)
	{
		m_aPointsOG.push_back(point->Sub(m_centerOfMass));
	}
}

void RigidBody::findBodyMassAndInertia(DOUBLE density, BOOL useCustumCOM, Vector3D custumCOM)
{
	m_mass = 0;

	for (std::shared_ptr<ConvexHull> hull : m_apHulls)
	{
		m_mass += hull->GetMass();
		Vector3D hullCOM = hull->GetCenterOfMass();
		m_centerOfMass.m_x += hullCOM.m_x * hull->GetMass();
		m_centerOfMass.m_y += hullCOM.m_y * hull->GetMass();
		m_centerOfMass.m_z += hullCOM.m_z * hull->GetMass();
	}

	m_inverseMass = 1.0 / m_mass;
	m_centerOfMass.m_x /= m_mass;
	m_centerOfMass.m_y /= m_mass;
	m_centerOfMass.m_z /= m_mass;

	if (useCustumCOM)
	{
		m_centerOfMass = custumCOM;
	}

	for (std::shared_ptr<ConvexHull> hull : m_apHulls)
	{
		Vector3D hullCOM = hull->GetCenterOfMass();
		Vector3D hullRel;
		hullRel.m_x = hullCOM.m_x - m_centerOfMass.m_x;
		hullRel.m_y = hullCOM.m_y - m_centerOfMass.m_y;
		hullRel.m_z = hullCOM.m_z - m_centerOfMass.m_z;
		Matrix33* hTens = hull->GetInertia();

		m_inertiaTensor.m_aaElements[0][0] += hTens->m_aaElements[0][0] + hull->GetMass() * (hullRel.m_y * hullRel.m_y + hullRel.m_z * hullRel.m_z);
		m_inertiaTensor.m_aaElements[0][1] += hTens->m_aaElements[0][1] - hull->GetMass() * (hullRel.m_x * hullRel.m_y);
		m_inertiaTensor.m_aaElements[0][2] += hTens->m_aaElements[0][2] - hull->GetMass() * (hullRel.m_x * hullRel.m_z);
		m_inertiaTensor.m_aaElements[1][0] += hTens->m_aaElements[1][0] - hull->GetMass() * (hullRel.m_x * hullRel.m_y);
		m_inertiaTensor.m_aaElements[1][1] += hTens->m_aaElements[1][1] + hull->GetMass() * (hullRel.m_x * hullRel.m_x + hullRel.m_z * hullRel.m_z);
		m_inertiaTensor.m_aaElements[1][2] += hTens->m_aaElements[1][2] - hull->GetMass() * (hullRel.m_z * hullRel.m_y);
		m_inertiaTensor.m_aaElements[2][0] += hTens->m_aaElements[2][0] - hull->GetMass() * (hullRel.m_x * hullRel.m_z);
		m_inertiaTensor.m_aaElements[2][1] += hTens->m_aaElements[2][1] - hull->GetMass() * (hullRel.m_z * hullRel.m_y);
		m_inertiaTensor.m_aaElements[2][2] += hTens->m_aaElements[2][2] + hull->GetMass() * (hullRel.m_x * hullRel.m_x + hullRel.m_y * hullRel.m_y);
	}

	m_tensorInverse = m_inertiaTensor.Invert();

	/*for (UINT i = 0; i < 9; ++i) {
		printf("%f, ", m_inertiaTensor[i]);
		if ((i + 1) % 3 == 0)
			printf("\n");
	}

	printf("%f, %f, %f\n", m_centerOfMass.m_x, m_centerOfMass.m_y, m_centerOfMass.m_z);

	printf("%f\n", m_mass);
	printf("____________________________________\n");*/
}

DOUBLE RigidBody::getInertiaOfAxis(const Vector3D axisIn)
{
	Vector3D axis = FindVectorRelativeToBodyFrame(axisIn).GetUnitVector();
	Vector3D iAxis = m_inertiaTensor * axis;

	return axis.DotProduct(iAxis);
}

void RigidBody::findCollisionRadius()
{
	DOUBLE radius = 0.0;
	for (Vector3D* point : m_apPointsToTransform)
	{
		DOUBLE dist = point->Sub(m_centerOfMass).GetMagnitude();
		if (dist > radius)
		{
			radius = dist;
		}
	}

	m_collisionRadiusSquared = radius * radius;
	m_collisionRadius = radius;
}

void RigidBody::findDimensions()
{
	DOUBLE maxX = -INFINITY;
	DOUBLE minX = INFINITY;
	DOUBLE maxY = -INFINITY;
	DOUBLE minY = INFINITY;
	DOUBLE maxZ = -INFINITY;
	DOUBLE minZ = INFINITY;

	for (Vector3D* point : m_apPointsToTransform)
	{
		maxX = fmax(point->m_x, maxX);
		minX = fmin(point->m_x, minX);
		maxY = fmax(point->m_y, maxY);
		minY = fmin(point->m_y, minY);
		maxZ = fmax(point->m_z, maxZ);
		minZ = fmin(point->m_z, minZ);
	}

	m_fixedDimensions = Vector3D(maxX - minX, maxY - minY, maxZ - minZ);
	m_dimCenter = Vector3D(maxX + minX, maxY + minY, maxZ + minZ).ScalarMultiply(0.5);
}

Vector3D RigidBody::gyroAccel(DOUBLE time)
{
	Vector3D wB = m_orientation.GetInverse().Rotate(m_angularVelocity);
	Vector3D w2B = wB;

	int nItr = 1;
	for (UINT i = 0; i < nItr; ++i)
	{
		Vector3D funcW = (m_inertiaTensor * w2B.Sub(wB)).Add(w2B.CrossProduct(m_inertiaTensor * w2B).ScalarMultiply(time));
		Matrix33 jacobian = m_inertiaTensor + (((Matrix33::Skew(w2B) * m_inertiaTensor) + (Matrix33::Skew(m_inertiaTensor * w2B) * -1)) * time);
		w2B = Matrix33::NewtonSolve(w2B, funcW, jacobian);
	}

	return m_orientation.Rotate(w2B);
}
