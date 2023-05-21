#include "ConvexHull.h"

ConvexHull::ColPointInfo::ColPointInfo(Vector3D point, DOUBLE penDepth)
	: m_point(point)
	, m_penDepth(penDepth)
{
}

ConvexHull::ColPointInfo::~ColPointInfo()
{
}

ConvexHull::Edge::Edge(Vector3D* p1, Vector3D* p2, BOOL bIsInteriorEdge)
	: m_p1(p1)
	, m_p2(p2)
	, m_inverseMagnitude(1.0 / Vector3D(*p1, *p2).GetMagnitude())
	, m_bIsInteriorEdge(bIsInteriorEdge)
{
}

ConvexHull::Edge::~Edge()
{
	delete m_p1;
	delete m_p2;
}

BOOL ConvexHull::Edge::operator==(const Edge& edge)
{
	return (*edge.m_p1 == *m_p1 && *edge.m_p2 == *m_p2) || (*edge.m_p2 == *m_p1 && *edge.m_p1 == *m_p2);
}

ConvexHull::ConvexHull(const std::vector<RigidSurface*>* surfaces, DOUBLE density)
	: m_apSurfaces(*surfaces)
	, m_apColPoints()
	, m_apColEdges()
	, m_centerOfMass()
	, m_collisionRadius()
	, m_collisionRadiusSquared()
	, m_mass()
	, m_inertiaTensor()
{
	findBodyMassAndInertia(density);
	findColPointsEdges();
	findCollisionRadius();
}

ConvexHull::ConvexHull(const ConvexHull& hull)
	: m_apSurfaces()
	, m_apColPoints()
	, m_apColEdges()
	, m_centerOfMass(hull.m_centerOfMass)
	, m_collisionRadius(hull.m_collisionRadius)
	, m_collisionRadiusSquared(hull.m_collisionRadiusSquared)
	, m_mass(hull.m_mass)
	, m_inertiaTensor(hull.m_inertiaTensor)
{
	for (RigidSurface* surface : hull.m_apSurfaces)
	{
		m_apSurfaces.push_back(new RigidSurface(*surface));
	}

	findColPointsEdges();
}

ConvexHull::~ConvexHull()
{
	for (RigidSurface* surface : m_apSurfaces)
	{
		delete surface;
	}
	for (Vector3D* point : m_apColPoints)
	{
		delete point;
	}
	for (Edge* edge : m_apColEdges)
	{
		delete edge;
	}
}

Vector3D ConvexHull::GetCenterOfMass()
{
	return m_centerOfMass;
}

Vector3D* ConvexHull::GetCOMPointer()
{
	return &m_centerOfMass;
}

std::vector<Vector3D*>* ConvexHull::GetColPoints()
{
	return &m_apColPoints;
}

std::vector<RigidSurface*>* ConvexHull::GetSurfaces()
{
	return &m_apSurfaces;
}

DOUBLE ConvexHull::GetCollisionRadius()
{
	return m_collisionRadius;
}

Matrix33* ConvexHull::GetInertia()
{
	return &m_inertiaTensor;
}

DOUBLE ConvexHull::GetMass()
{
	return m_mass;
}

BOOL ConvexHull::IsHullsInCollisionRange(std::shared_ptr<ConvexHull> hull)
{
	DOUBLE xDist = m_centerOfMass.m_x - hull->m_centerOfMass.m_x;
	DOUBLE yDist = m_centerOfMass.m_y - hull->m_centerOfMass.m_y;
	DOUBLE zDist = m_centerOfMass.m_z - hull->m_centerOfMass.m_z;

	DOUBLE sqrDist = xDist * xDist + yDist * yDist + zDist * zDist;
	DOUBLE colDist = (m_collisionRadius + hull->GetCollisionRadius()) * (m_collisionRadius + hull->GetCollisionRadius());

	return sqrDist <= colDist;
}

BOOL ConvexHull::SATColliderDetect(std::shared_ptr<ConvexHull> potCollider, std::vector<ColPointInfo>* colSupPoints, Vector3D* collisionPoint, Vector3D* nVect, DOUBLE* collisionDepth, BOOL* separatingAxis)
{
	std::vector<Vector3D> testedDirs;

	Vector3D colPoint;
	Vector3D colVector;
	DOUBLE lowestColDepth = -1;
	DOUBLE winningCollideeMax;
	*separatingAxis = FALSE;
	BOOL bIsColFound = FALSE;

	for (RigidSurface* surface : m_apSurfaces) {

		Vector3D n = surface->GetUnitNorm();

		BOOL alreadyTested = FALSE;
		for (Vector3D dir : testedDirs) {
			if (abs(n.DotProduct(dir)) > 0.99) {
				alreadyTested = TRUE;
			}
		}
		if (alreadyTested) {
			continue;
		}
		if (!surface->IsInteriorSurface()) {
			testedDirs.push_back(n);
		}

		DOUBLE colliderMax, colliderMin, collideeMax, collideeMin;

		Vector3D minPoint;
		Vector3D maxPoint;

		findMaxMin(n, &collideeMax, &collideeMin, nullptr, nullptr);
		potCollider->findMaxMin(n, &colliderMax, &colliderMin, &maxPoint, &minPoint);

		DOUBLE colDepth;

		if (!(colliderMin > collideeMax || colliderMax < collideeMin))	//check for intersection
		{
			if (!surface->IsInteriorSurface())
			{
				colDepth = collideeMax - colliderMin;
				Vector3D potColPoint = minPoint;
				BOOL flipped = FALSE;
				if (colliderMax - collideeMin < colDepth)
				{
					flipped = TRUE;
					colDepth = colliderMax - collideeMin;
					n = n.GetInverse();
					potColPoint = maxPoint;
				}

				if ((TRUE || IsPointInsideBody(potColPoint)) && (colDepth < lowestColDepth || lowestColDepth == -1))
				{
					lowestColDepth = colDepth;
					colPoint = potColPoint;
					colVector = n;
					winningCollideeMax = (flipped) ? -collideeMin : collideeMax;
					bIsColFound = TRUE;
				}
			}
		}
		else
		{
			//existense of separating axis => no colliding
			DOUBLE d1 = abs(colliderMin - collideeMax);
			DOUBLE d2 = abs(collideeMin - colliderMax);
			DOUBLE colDepth = (d1 < d2) ? d1 : d2;
			if (colDepth < lowestColDepth)
			{
				lowestColDepth = colDepth;
			}
			*separatingAxis = TRUE;
			return FALSE;
		}
	}

	*collisionDepth = lowestColDepth;

	if (*separatingAxis)
	{
		return FALSE;
	}

	Vector3D colSurfaceN;
	RigidSurface* colSurface = nullptr;
	for (RigidSurface* surface : m_apSurfaces)
	{
		Vector3D norm = surface->GetUnitNorm();
		if (colSurface == nullptr || norm.DotProduct(colVector) > colSurfaceN.DotProduct(colVector))
		{
			colSurface = surface;
			colSurfaceN = norm;
		}
	}

	/*if (lowestColDepth != -1)
	{
		for (Vector3D* point : potCollider->m_apColPoints)
		{
			if (IsPointInsideBody(*point))
			{
				DOUBLE nDotVal = point->DotProduct(colVector);
				colSupPoints->push_back(ColPointInfo(*point, winningCollideeMax - nDotVal));
			}
		}
	}*/

	//check colPoint in clipped manifold
	static DOUBLE tolerance = 0;
	if (bIsColFound)
	{
		for (UINT pointIndex = 0; pointIndex < colSurface->GetPoints()->size(); ++pointIndex)
		{
			Vector3D* p1 = colSurface->GetPoints()->at(pointIndex);
			int nextPointIndex = (pointIndex == colSurface->GetPoints()->size() - 1) ? 0 : pointIndex + 1;
			Vector3D* p2 = colSurface->GetPoints()->at(nextPointIndex);
			Vector3D toP = colPoint.Sub(*p1);
			Vector3D clipNorm = colVector.CrossProduct(p2->Sub(*p1));
			if (clipNorm.DotProduct(toP) < -tolerance)
			{
				return FALSE;
			}
		}
	}
	else
	{
		return FALSE;
	}

	*collisionPoint = colPoint;
	*nVect = colVector;

	return bIsColFound;
}

BOOL ConvexHull::SATEdgeCol(std::shared_ptr<ConvexHull> potCollider, Vector3D* collisionPoint, Vector3D* nVect, DOUBLE* collisionDepth, BOOL* separatingAxis)
{
	std::vector<Vector3D> testedDirs;

	Vector3D colPoint;
	Vector3D colVector;
	DOUBLE lowestColDepth = -1;

	BOOL separtingAxisFound = FALSE;

	*separatingAxis = FALSE;
	for (Edge* edge1 : m_apColEdges)
	{

		for (Edge* edge2 : potCollider->m_apColEdges)
		{
			Vector3D n = Vector3D(*edge1->m_p1, *edge1->m_p2).CrossProduct(Vector3D(*edge2->m_p1, *edge2->m_p2));
			if (n.IsZero())
			{
				continue;	// vectors parralel, cant find normal
			}
			n = n.GetUnitVector();

			DOUBLE colliderMax, colliderMin, collideeMax, collideeMin;
			BOOL colliderFirst = TRUE;
			BOOL collideeFirst = TRUE;

			findMaxMin(n, &collideeMax, &collideeMin, nullptr, nullptr);
			potCollider->findMaxMin(n, &colliderMax, &colliderMin, nullptr, nullptr);

			DOUBLE colDepth;
			if (!(colliderMin > collideeMax || colliderMax < collideeMin))	//check for intersection
			{
				if (!edge1->m_bIsInteriorEdge && !edge2->m_bIsInteriorEdge)
				{
					colDepth = collideeMax - colliderMin;
					if (colliderMax - collideeMin < colDepth)
					{
						colDepth = colliderMax - collideeMin;
						n = n.GetInverse();
					}

					if (colDepth < lowestColDepth || lowestColDepth == -1)
					{
						Vector3D e1Axis = Vector3D(*edge1->m_p1, *edge1->m_p2).ScalarMultiply(edge1->m_inverseMagnitude);
						Vector3D e1Norm = e1Axis.CrossProduct(n);

						Vector3D e2p1Rel = Vector3D(*edge1->m_p1, *edge2->m_p1);
						Vector3D e2p2Rel = Vector3D(*edge1->m_p1, *edge2->m_p2);

						DOUBLE p1e1Val = e2p1Rel.DotProduct(e1Axis);
						DOUBLE p2e1Val = e2p2Rel.DotProduct(e1Axis);
						DOUBLE p1NormVal = e2p1Rel.DotProduct(e1Norm);
						DOUBLE p2NormVal = e2p2Rel.DotProduct(e1Norm);

						if (p2NormVal - p1NormVal == 0)
						{
							continue;	//edges are parralel, collision can't occur
						}

						DOUBLE p1ToP2e1Slope = (p2e1Val - p1e1Val) / (p2NormVal - p1NormVal);
						DOUBLE intersectE1AxisVal = p1e1Val + p1ToP2e1Slope * (-p1NormVal);
						if (intersectE1AxisVal < 0 || intersectE1AxisVal >(1.0 / edge1->m_inverseMagnitude))
						{
							continue;	//edges are offset and couldnt collide
						}

						Vector3D* p1 = edge1->m_p1;
						Vector3D e1ToColP = e1Axis.ScalarMultiply(intersectE1AxisVal);
						colPoint = Vector3D(p1->m_x + e1ToColP.m_x, p1->m_y + e1ToColP.m_y, p1->m_z + e1ToColP.m_z);
						colVector = n;
						lowestColDepth = colDepth;
					}
				}
			}
			else
			{
				//existense of separating axis => no colliding
				DOUBLE d1 = abs(colliderMin - collideeMax);
				DOUBLE d2 = abs(collideeMin - colliderMax);
				DOUBLE colDepth = (d1 < d2) ? d1 : d2;
				if (colDepth < lowestColDepth)
				{
					lowestColDepth = colDepth;
				}
				*separatingAxis = TRUE;
				return FALSE;
			}
		}
	}

	*collisionDepth = lowestColDepth;

	if (*separatingAxis)
	{
		return FALSE;
	}

	*collisionPoint = colPoint;
	*nVect = colVector;
	return TRUE;
}

BOOL ConvexHull::IsPointInsideBody(const Vector3D point)
{
	for (RigidSurface* surface : m_apSurfaces)
	{
		Vector3D normalVector = surface->GetUnitNorm();
		Vector3D posToPoint(*(surface->GetPoints()->at(0)), point);
		if (normalVector.DotProduct(posToPoint) > 0)
		{
			return FALSE;
		}
	}
	return TRUE;
}

enum axis { X, Y, Z };

DOUBLE getAxisVal(Vector3D* p, enum axis axis)
{
	switch (axis) {
	case X:
		return p->m_x;
		break;
	case Y:
		return p->m_y;
		break;
	case Z:
		return p->m_z;
		break;
	}
}

// Fast and Accurate Computation of Polyhedral Mass Properties (Brian Miritch)
void ConvexHull::findBodyMassAndInertia(DOUBLE density)
{
	DOUBLE v = 0, vX = 0, vY = 0, vZ = 0, vXSqrd = 0, vYSqrd = 0, vZSqrd = 0;
	DOUBLE vXY = 0, vXZ = 0, vYZ = 0;

	for (RigidSurface* s : m_apSurfaces)
	{
		DOUBLE sXSqrd, sYSqrd, sZSqrd, sXCbd, sYCbd, sZCbd;
		DOUBLE sXSqrdY, sXSqrdZ, sYSqrdZ, sX, u;

		DOUBLE* sA, * sB, * sC, * sASqrd, * sBSqrd, * sCSqrd;
		DOUBLE* sACbd, * sBCbd, * sCCbd, * sASqrdB, * sBSqrdC, * sASqrdC, * sBSqrdA, * sCSqrdA, * sCSqrdB;

		DOUBLE f = 0, fA = 0, fB = 0, fAB = 0, fASqrd = 0, fBSqrd = 0, fASqrdB = 0, fBSqrdA = 0;
		DOUBLE fACbd = 0, fBCbd = 0;

		enum axis A;	// horz axis
		enum axis B;	// vert axis
		enum axis C;	// normal axis

		DOUBLE nA;
		DOUBLE nB;
		DOUBLE nC;
		DOUBLE invC;

		Vector3D normV = s->GetUnitNorm();

		if (abs(normV.m_x) > abs(normV.m_y) && abs(normV.m_x) > abs(normV.m_z))
		{
			C = X;
			A = Y;
			B = Z;
			nC = normV.m_x;
			nA = normV.m_y;
			nB = normV.m_z;

			sA = &u;
			sB = &u;
			sC = &sX;
			sASqrd = &sYSqrd;
			sBSqrd = &sZSqrd;
			sCSqrd = &sXSqrd;
			sACbd = &sYCbd;
			sBCbd = &sZCbd;
			sCCbd = &sXCbd;
			sASqrdB = &sYSqrdZ;
			sCSqrdB = &sXSqrdZ;
			sCSqrdA = &sXSqrdY;
			sBSqrdA = &u;
			sBSqrdC = &u;
			sASqrdC = &u;
			sASqrd = &sYSqrd;
			sBSqrd = &sZSqrd;
			sCSqrd = &sXSqrd;
		}
		else if (abs(normV.m_y) > abs(normV.m_z))
		{
			C = Y;
			A = Z;
			B = X;
			nC = normV.m_y;
			nA = normV.m_z;
			nB = normV.m_x;

			sA = &u;
			sB = &sX;
			sC = &u;
			sASqrd = &sZSqrd;
			sBSqrd = &sXSqrd;
			sCSqrd = &sYSqrd;
			sACbd = &sZCbd;
			sBCbd = &sXCbd;
			sCCbd = &sYCbd;
			sBSqrdA = &sXSqrdZ;
			sBSqrdC = &sXSqrdY;
			sCSqrdA = &sYSqrdZ;
			sASqrdB = &u;
			sASqrdC = &u;
			sCSqrdB = &u;
			sASqrd = &sZSqrd;
			sBSqrd = &sXSqrd;
			sCSqrd = &sYSqrd;
		}
		else
		{
			C = Z;
			A = X;
			B = Y;
			nC = normV.m_z;
			nA = normV.m_x;
			nB = normV.m_y;

			sA = &sX;
			sB = &u;
			sC = &u;
			sASqrd = &sXSqrd;
			sBSqrd = &sYSqrd;
			sCSqrd = &sZSqrd;
			sACbd = &sXCbd;
			sBCbd = &sYCbd;
			sCCbd = &sZCbd;
			sASqrdB = &sXSqrdY;
			sBSqrdC = &sYSqrdZ;
			sASqrdC = &sXSqrdZ;
			sBSqrdA = &u;
			sCSqrdA = &u;
			sCSqrdB = &u;
			sASqrd = &sXSqrd;
			sBSqrd = &sYSqrd;
			sCSqrd = &sZSqrd;
		}

		for (UINT i = 0; i < s->GetPoints()->size(); ++i)
		{
			Vector3D* p1 = s->GetPoints()->at(i);
			Vector3D* p2 = i + 1 == s->GetPoints()->size() ? s->GetPoints()->at(0) : s->GetPoints()->at(i + 1);

			DOUBLE a1 = getAxisVal(p1, A);
			DOUBLE b1 = getAxisVal(p1, B);
			DOUBLE a2 = getAxisVal(p2, A);
			DOUBLE b2 = getAxisVal(p2, B);

			DOUBLE a21 = a2 - a1;
			DOUBLE b21 = b2 - b1;

			f += b21 * (a1 + a21 / 2);
			fA += 0.5 * b21 * (a1 * a1 + a1 * a21 + a21 * a21 / 3);
			fB += -0.5 * a21 * (b1 * b1 + b1 * b21 + b21 * b21 / 3);
			fAB += 0.5 * b21 * (b1 * (a1 * a1 + a1 * a21 + a21 * a21 / 3) + b21 * (a1 * a1 / 2.0 + 2 * a1 * a21 / 3.0 + a21 * a21 / 4.0));
			fASqrd += (1 / 3.0) * b21 * (a1 * a1 * a1 + 3 * a1 * a1 * a21 / 2.0 + a1 * a21 * a21 + a21 * a21 * a21 / 4.0);
			fBSqrd += -(1 / 3.0) * a21 * (b1 * b1 * b1 + 3 * b1 * b1 * b21 / 2.0 + b1 * b21 * b21 + b21 * b21 * b21 / 4.0);
			fASqrdB += (1 / 3.0) * b21 * (b1 * (a1 * a1 * a1 + 3 * a1 * a1 * a21 / 2.0 + a1 * a21 * a21 + a21 * a21 * a21 / 4.0)
				+ b21 * (a1 * a1 * a1 / 2.0 + a1 * a1 * a21 + 3 * a1 * a21 * a21 / 4.0 + a21 * a21 * a21 / 5.0));
			fBSqrdA += -(1 / 3.0) * a21 * (a1 * (b1 * b1 * b1 + 3 * b1 * b1 * b21 / 2.0 + b1 * b21 * b21 + b21 * b21 * b21 / 4.0)
				+ a21 * (b1 * b1 * b1 / 2.0 + b1 * b1 * b21 + 3 * b1 * b21 * b21 / 4.0 + b21 * b21 * b21 / 5.0));
			fACbd += 0.25 * b21 * (a1 * a1 * a1 * a1 + 2 * a1 * a1 * a1 * a21 + 2 * a1 * a1 * a21 * a21 + a1 * a21 * a21 * a21 + a21 * a21 * a21 * a21 / 5.0);
			fBCbd += -0.25 * a21 * (b1 * b1 * b1 * b1 + 2 * b1 * b1 * b1 * b21 + 2 * b1 * b1 * b21 * b21 + b1 * b21 * b21 * b21 + b21 * b21 * b21 * b21 / 5.0);
		}

		invC = 1 / nC;
		//DOUBLE absInvC = abs(invC);
		Vector3D* p = s->GetPoints()->at(0);
		DOUBLE k = -(normV.m_x * p->m_x + normV.m_y * p->m_y + normV.m_z * p->m_z);

		*sA = invC * fA;
		*sB = invC * fB;
		*sC = -invC * invC * (f * k + nA * fA + nB * fB);
		*sASqrd = invC * fASqrd;
		*sBSqrd = invC * fBSqrd;
		*sCSqrd = invC * invC * invC * (k * k * f + nA * nA * fASqrd + nB * nB * fBSqrd + 2 * k * nA * fA
			+ 2 * k * nB * fB + 2 * nA * nB * fAB);
		*sASqrdB = invC * fASqrdB;
		*sBSqrdA = invC * fBSqrdA;
		*sASqrdC = -invC * invC * (k * fASqrd + nA * fACbd + nB * fASqrdB);
		*sBSqrdC = -invC * invC * (k * fBSqrd + nB * fBCbd + nA * fBSqrdA);
		*sCSqrdA = invC * invC * invC * (k * k * fA + nA * nA * fACbd + nB * nB * fBSqrdA + 2 * k * nA * fASqrd + 2 * k * nB * fAB + 2 * nA * nB * fASqrdB);
		*sCSqrdB = invC * invC * invC * (k * k * fB + nA * nA * fASqrdB + nB * nB * fBCbd + 2 * k * nA * fAB + 2 * k * nB * fBSqrd + 2 * nA * nB * fBSqrdA);
		*sACbd = invC * fACbd;
		*sBCbd = invC * fBCbd;
		*sCCbd = -invC * invC * invC * invC * (k * k * k * f + nA * nA * nA * fACbd + nB * nB * nB * fBCbd + 3 * k * nA * nA * fASqrd + 3 * k * nB * nB * fBSqrd +
			3 * nA * nA * nB * fASqrdB + 3 * nA * nB * nB * fBSqrdA + 3 * k * k * nA * fA + 3 * k * k * nB * fB + 6 * k * nA * nB * fAB);

		v += normV.m_x * sX;
		vX += 0.5 * normV.m_x * sXSqrd;
		vY += 0.5 * normV.m_y * sYSqrd;
		vZ += 0.5 * normV.m_z * sZSqrd;
		vXSqrd += (1.0 / 3.0) * normV.m_x * sXCbd;
		vYSqrd += (1.0 / 3.0) * normV.m_y * sYCbd;
		vZSqrd += (1.0 / 3.0) * normV.m_z * sZCbd;
		vXY += 0.5 * normV.m_x * sXSqrdY;
		vXZ += 0.5 * normV.m_x * sXSqrdZ;
		vYZ += 0.5 * normV.m_y * sYSqrdZ;

	}

	m_mass = v * density;
	m_centerOfMass.m_x = vX / v;
	m_centerOfMass.m_y = vY / v;
	m_centerOfMass.m_z = vZ / v;

	m_inertiaTensor.m_aaElements[0][0] = density * (vYSqrd + vZSqrd - v * (m_centerOfMass.m_y * m_centerOfMass.m_y + m_centerOfMass.m_z * m_centerOfMass.m_z));
	m_inertiaTensor.m_aaElements[0][1] = -density * (vXY - v * m_centerOfMass.m_x * m_centerOfMass.m_y);
	m_inertiaTensor.m_aaElements[0][2] = -density * (vXZ - v * m_centerOfMass.m_x * m_centerOfMass.m_z);
	m_inertiaTensor.m_aaElements[1][1] = density * (vXSqrd + vZSqrd - v * (m_centerOfMass.m_x * m_centerOfMass.m_x + m_centerOfMass.m_z * m_centerOfMass.m_z));
	m_inertiaTensor.m_aaElements[1][2] = -density * (vYZ - v * m_centerOfMass.m_y * m_centerOfMass.m_z);
	m_inertiaTensor.m_aaElements[2][2] = density * (vXSqrd + vYSqrd - v * (m_centerOfMass.m_x * m_centerOfMass.m_x + m_centerOfMass.m_y * m_centerOfMass.m_y));
	m_inertiaTensor.m_aaElements[1][0] = m_inertiaTensor.m_aaElements[0][1];
	m_inertiaTensor.m_aaElements[1][2] = m_inertiaTensor.m_aaElements[0][2];
	m_inertiaTensor.m_aaElements[2][1] = m_inertiaTensor.m_aaElements[1][2];


	/*for (UINT i = 0; i < 9; ++i) {
		printf("%f, ", m_inertiaTensor[i]);
		if ((i + 1) % 3 == 0)
			printf("\n");
	}

	printf("%f, %f, %f\n", m_centerOfMass.m_x, m_centerOfMass.m_y, m_centerOfMass.m_z);

	printf("%f\n", m_mass);
	printf("____________________________________\n");*/
}

void ConvexHull::findCollisionRadius()
{
	DOUBLE greatestRadiusSquared = 0;
	for (Vector3D* point : m_apColPoints)
	{
		DOUBLE squaredRadius = (m_centerOfMass.m_x - point->m_x) * (m_centerOfMass.m_x - point->m_x) + (m_centerOfMass.m_y - point->m_y) * (m_centerOfMass.m_y - point->m_y)
			+ (m_centerOfMass.m_z - point->m_z) * (m_centerOfMass.m_z - point->m_z);
		if (squaredRadius > greatestRadiusSquared)
		{
			greatestRadiusSquared = squaredRadius;
		}
	}

	m_collisionRadiusSquared = greatestRadiusSquared;
	m_collisionRadius = sqrt(greatestRadiusSquared);
}

void ConvexHull::findColPointsEdges()
{
	for (RigidSurface* surface : m_apSurfaces)
	{
		Vector3D* prevPoint = nullptr;
		Vector3D* p0 = nullptr;

		for (UINT pointIndex = 0; pointIndex < surface->GetPoints()->size(); ++pointIndex)
		{
			Vector3D* point = surface->GetPoints()->at(pointIndex);
			BOOL pAlreadyAdded = FALSE;
			for (Vector3D* colPoint : m_apColPoints)
			{
				if (*colPoint == *point)
				{
					pAlreadyAdded = TRUE;
					point = colPoint;
					break;
				}
			}
			if (!pAlreadyAdded)
			{
				m_apColPoints.push_back(point);
			}

			if (prevPoint != nullptr)
			{
				BOOL edgeAlreadyAdded = FALSE;
				Edge edge = Edge(point, prevPoint, surface->IsInteriorSurface());
				for (Edge* existingEdge : m_apColEdges)
				{
					if (*existingEdge == edge)
					{
						if (!surface->IsInteriorSurface() && existingEdge->m_bIsInteriorEdge)
						{
							existingEdge->m_bIsInteriorEdge = FALSE;
						}
						edgeAlreadyAdded = TRUE;
						break;
					}
				}
				if (!edgeAlreadyAdded)
				{
					m_apColEdges.push_back(new Edge(edge));
				}
			}
			prevPoint = point;
			if (pointIndex == 0)
			{
				p0 = point;
			}

			if (pointIndex == surface->GetPoints()->size() - 1)
			{
				BOOL edgeAlreadyAdded = FALSE;
				Edge edge = Edge(point, p0, surface->IsInteriorSurface());
				for (Edge* existingEdge : m_apColEdges)
				{
					if (*existingEdge == edge)
					{
						if (!surface->IsInteriorSurface() && existingEdge->m_bIsInteriorEdge)
						{
							existingEdge->m_bIsInteriorEdge = FALSE;
						}
						edgeAlreadyAdded = TRUE;
						break;
					}
				}
				if (!edgeAlreadyAdded)
				{
					m_apColEdges.push_back(new Edge(edge));
				}
			}
		}
	}
}

void ConvexHull::findMaxMin(Vector3D n, DOUBLE* max, DOUBLE* min, Vector3D* maxP, Vector3D* minP)
{
	DOUBLE nMax, nMin;
	BOOL init = TRUE;
	Vector3D* minPoint = nullptr;
	Vector3D* maxPoint = nullptr;

	for (Vector3D* point : m_apColPoints)
	{
		DOUBLE nDotVal = point->DotProduct(n);
		if (init)
		{
			nMin = nDotVal;
			nMax = nDotVal;
			minPoint = point;
			maxPoint = point;
			init = FALSE;
		}
		else
		{
			if (nDotVal < nMin)
			{
				nMin = nDotVal;
				minPoint = point;
			}
			else if (nDotVal > nMax)
			{
				nMax = nDotVal;
				maxPoint = point;
			}
		}
	}

	*max = nMax;
	*min = nMin;
	if (maxP != nullptr)
	{
		*maxP = *maxPoint;
	}
	if (minP != nullptr)
	{
		*minP = *minPoint;
	}
}
