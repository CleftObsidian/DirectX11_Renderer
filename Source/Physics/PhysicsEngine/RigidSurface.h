#pragma once

#include "Math/Vector3D.h"

class RigidSurface
{
public:
	RigidSurface() = delete;
	RigidSurface(const std::vector<Vector3D>* points, const Vector3D normalVector, BOOL interiorSurface = FALSE);
	RigidSurface(const RigidSurface& surface);
	~RigidSurface();

	std::vector<Vector3D*>* GetPoints();
	Vector3D GetUnitNorm();
	DOUBLE GetInverseSegmentMagnitude(int pointIndex);	//returns magntiude of line from point i to point i + 1 in points
	BOOL IsInteriorSurface();

private:
	Vector3D getNCross();

	void caclulateInverseSegmentMagnitudes();
	void reverseOrder();

private:
	std::vector<Vector3D*> m_points;
	std::vector<DOUBLE> m_inverseSegmentMagnitudes;
	DOUBLE m_nVInverseMagnitude;
	BOOL m_interiorSurface;
};