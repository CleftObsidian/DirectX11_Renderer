#pragma once

#include "Rotor.h"

class Transformation3D
{
public:
	static void TranslatePoints(std::vector<Vector3D*>* points, const Vector3D translation) {
		TranslatePoints(points, translation.m_x, translation.m_y, translation.m_z);
	}

	static void TranslatePoint(Vector3D* p, const Vector3D translation) {
		TranslatePoint(p, translation.m_x, translation.m_y, translation.m_z);
	}

	static void TranslatePoint(Vector3D* p, DOUBLE x, DOUBLE y, DOUBLE z) {
		p->m_x += x;
		p->m_y += y;
		p->m_z += z;
	}

	static void TranslatePoints(std::vector<Vector3D*>* points, DOUBLE x, DOUBLE y, DOUBLE z) {
		for (Vector3D* p : *points)
		{
			TranslatePoint(p, x, y, z);
		}
	}

	static void RotatePointAroundZParallelAxis(Vector3D* p, DOUBLE theta, DOUBLE xCoord, DOUBLE yCoord, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//x' = xcos(a) - ysin(a)
		//y' = xsin(a) + ycos(a)
		DOUBLE relativeX = p->m_x - xCoord;
		DOUBLE relativeY = p->m_y - yCoord;
		p->m_x = xCoord + relativeX * cosTheta - relativeY * sinTheta;
		p->m_y = yCoord + relativeX * sinTheta + relativeY * cosTheta;
	}

	static void RotatePointsAroundZParallelAxis(std::vector<Vector3D*>* points, DOUBLE theta, DOUBLE xCoord, DOUBLE yCoord, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Vector3D* p : *points)
		{
			RotatePointAroundZParallelAxis(p, theta, xCoord, yCoord, cosTheta, sinTheta);
		}
	}

	static void RotatePointAroundXParralelAxis(Vector3D* p, DOUBLE theta, DOUBLE yCoord, DOUBLE zCoord, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//x' = xcos(a) - ysin(a)
		//y' = xsin(a) + ycos(a)
		DOUBLE relativeY = p->m_y - yCoord;
		DOUBLE relativeZ = p->m_z - zCoord;
		p->m_y = yCoord + relativeY * cosTheta - relativeZ * sinTheta;
		p->m_z = zCoord + relativeY * sinTheta + relativeZ * cosTheta;
	}

	static void RotatePointsAroundXParralelAxis(std::vector<Vector3D*>* points, DOUBLE theta, DOUBLE yCoord, DOUBLE zCoord, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Vector3D* p : *points)
		{
			RotatePointAroundXParralelAxis(p, theta, yCoord, zCoord, cosTheta, sinTheta);
		}
	}

	static void RotatePointAroundYParralelAxis(Vector3D* p, DOUBLE theta, DOUBLE xCoord, DOUBLE zCoord, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//x' = xcos(a) - ysin(a)
		//y' = xsin(a) + ycos(a)
		DOUBLE relativeX = p->m_x - xCoord;
		DOUBLE relativeZ = p->m_z - zCoord;
		p->m_x = xCoord + relativeX * cosTheta + relativeZ * sinTheta;
		p->m_z = zCoord - relativeX * sinTheta + relativeZ * cosTheta;
	}

	static void RotatePointsAroundYParralelAxis(std::vector<Vector3D*>* points, DOUBLE theta, DOUBLE xCoord, DOUBLE zCoord, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Vector3D* p : *points)
		{
			RotatePointAroundYParralelAxis(p, theta, xCoord, zCoord, cosTheta, sinTheta);
		}
	}

	static void RotatePointAroundArbitraryAxis(Vector3D* p, Vector3D* output, const Vector3D axisUnitVector, DOUBLE xPos, DOUBLE yPos, DOUBLE zPos, DOUBLE theta, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}

		// Rodrigues' rotation formula 
		/*Vector3D pointVector(p->m_x - xPos, p->m_y - yPos, p->m_z - zPos);
		Vector3D vParralel = axisUnitVector.ScalarMultiply(pointVector.DotProduct(axisUnitVector));
		Vector3D vPerpendicular = pointVector.Sub(vParralel);
		Vector3D crossedVector = axisUnitVector.CrossProduct(pointVector);
		Vector3D comp1 = vPerpendicular.ScalarMultiply(cosTheta);
		Vector3D comp2 = crossedVector.ScalarMultiply(sinTheta);
		Vector3D newPointVector = comp1.Add(comp2).Add(vParralel);
		output->m_x = newPointVector.m_x + xPos;
		output->m_y = newPointVector.m_y + yPos;
		output->m_z = newPointVector.m_z + zPos;*/

		Vector3D pointVector(p->m_x - xPos, p->m_y - yPos, p->m_z - zPos);
		*output = Rotor(axisUnitVector, theta).Rotate(pointVector).Add(Vector3D(xPos, yPos, zPos));
	}

	static void RotatePointsAroundArbitraryAxis(std::vector<Vector3D*>* points, const Vector3D axisUnitVector, DOUBLE xPos, DOUBLE yPos, DOUBLE zPos, DOUBLE theta, DOUBLE cosTheta = -1, DOUBLE sinTheta = -1) {
		//axis vector should be unit length
		if (cosTheta == sinTheta)
		{
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Vector3D* p : *points)
		{
			RotatePointAroundArbitraryAxis(p, p, axisUnitVector, xPos, yPos, zPos, theta, cosTheta, sinTheta);
		}
	}
};