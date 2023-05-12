#include "RigidSurface.h"

RigidSurface::RigidSurface(const std::vector<Vector3D>* points, const Vector3D normalVector, BOOL interiorSurface)
	: m_apPoints()
	, m_aInverseSegmentMagnitudes()
	, m_nVInverseMagnitude()
	, m_bIsInteriorSurface(interiorSurface)
{
	//normalVector should have unit length
	for (Vector3D point : *points)
	{
		this->m_apPoints.push_back(new Vector3D(point.m_x, point.m_y, point.m_z));
	}

	caclulateInverseSegmentMagnitudes();

	Vector3D cross = getNCross();
	if (cross.DotProduct(normalVector) < 0)
	{
		reverseOrder();
		cross = getNCross();
	}
	m_nVInverseMagnitude = 1 / cross.GetMagnitude();	// ((cross.dotProduct(normalVector) < 0) ? -1 : 1) / cross.getMagnitude();
}

RigidSurface::RigidSurface(const RigidSurface& surface)
	: m_apPoints()
	, m_aInverseSegmentMagnitudes(surface.m_aInverseSegmentMagnitudes)
	, m_nVInverseMagnitude(surface.m_nVInverseMagnitude)
	, m_bIsInteriorSurface(surface.m_bIsInteriorSurface)
{
	for (Vector3D* point : surface.m_apPoints)
	{
		this->m_apPoints.push_back(new Vector3D(point->m_x, point->m_y, point->m_z));
	}
}

RigidSurface::~RigidSurface()
{
	for (Vector3D* point : m_apPoints)
	{
		delete point;
	}
}

std::vector<Vector3D*>* RigidSurface::GetPoints()
{
	return &m_apPoints;
}

Vector3D RigidSurface::GetUnitNorm()
{
	return getNCross().ScalarMultiply(m_nVInverseMagnitude);
}

DOUBLE RigidSurface::GetInverseSegmentMagnitude(int pointIndex)
{
	if (pointIndex < m_aInverseSegmentMagnitudes.size())
	{
		return m_aInverseSegmentMagnitudes.at(pointIndex);
	}
	else
	{
		return -1.0;
	}
}

BOOL RigidSurface::IsInteriorSurface()
{
	return m_bIsInteriorSurface;
}

Vector3D RigidSurface::getNCross()
{
	return Vector3D(*m_apPoints.at(0), *m_apPoints.at(1)).CrossProduct(Vector3D(*m_apPoints.at(1), *m_apPoints.at(2)));
}

void RigidSurface::caclulateInverseSegmentMagnitudes()
{
	for (UINT pointIndex = 0; pointIndex < m_apPoints.size(); ++pointIndex)
	{
		Vector3D* p1 = m_apPoints.at(pointIndex);
		Vector3D* p2 = (pointIndex == m_apPoints.size() - 1) ? m_apPoints.at(0) : m_apPoints.at(pointIndex + 1);
		Vector3D p1p2(*p1, *p2);
		m_aInverseSegmentMagnitudes.push_back(1.0 / p1p2.GetMagnitude());
	}
}

void RigidSurface::reverseOrder()
{
	//std::vector<Vector3D*> reverse;
	//for (UINT pointIndex = m_aPoints.size() - 1; pointIndex >= 0; --pointIndex) {
	//	reverse.push_back(m_aPoints.at(pointIndex));
	//}
	//m_aPoints = reverse;

	std::reverse(m_apPoints.begin(), m_apPoints.end());
}
