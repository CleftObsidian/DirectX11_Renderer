#pragma once

#include "RigidSurface.h"
#include "Math/Matrix33.h"

class ConvexHull
{
public:
	class ColPointInfo
	{
	public:
		ColPointInfo() = delete;
		ColPointInfo(Vector3D point, DOUBLE penDepth);
		~ColPointInfo();

	public:
		Vector3D m_point;
		DOUBLE m_penDepth;
	};

	class Edge
	{
	public:
		Edge() = delete;
		Edge(Vector3D* p1, Vector3D* p2, BOOL bIsInteriorEdge);
		~Edge();

		BOOL operator==(const Edge& edge);

	public:
		Vector3D* m_p1;
		Vector3D* m_p2;
		DOUBLE m_inverseMagnitude;
		BOOL m_bIsInteriorEdge;
	};

public:
	ConvexHull() = delete;
	ConvexHull(const std::vector<RigidSurface*>* surfaces, DOUBLE density);
	ConvexHull(const ConvexHull& hull);
	~ConvexHull();

	Vector3D GetCenterOfMass();
	Vector3D* GetCOMPointer();
	std::vector<Vector3D*>* GetColPoints();
	std::vector<RigidSurface*>* GetSurfaces();
	DOUBLE GetCollisionRadius();
	Matrix33* GetInertia();
	DOUBLE GetMass();
	BOOL IsHullsInCollisionRange(ConvexHull* hull);
	BOOL SATColliderDetect(ConvexHull* potCollider, std::vector<ColPointInfo>* colSupPoints, Vector3D* collisionPoint, Vector3D* nVect, DOUBLE* collisionDepth, BOOL* separatingAxis);
	//BOOL SATNew(ConvexHull* potCollider, std::vector<ColPointInfo>* colSupPoints, Vector3D* collisionPoint, Vector3D* nVect, DOUBLE* colDepth, BOOL* separatingAxis);
	BOOL SATEdgeCol(ConvexHull* potCollider, Vector3D* collisionPoint, Vector3D* nVect, DOUBLE* collisionDepth, BOOL* separatingAxis);
	BOOL IsPointInsideBody(const Vector3D point);

private:
	void findBodyMassAndInertia(DOUBLE density);
	void findCollisionRadius();
	void findColPointsEdges();
	void findMaxMin(Vector3D n, DOUBLE* max, DOUBLE* min, Vector3D* maxP, Vector3D* minP);

private:
	std::vector<RigidSurface*> m_apSurfaces;
	std::vector<Vector3D*> m_apColPoints;
	std::vector<Edge*> m_apColEdges;
	Vector3D m_centerOfMass;
	DOUBLE m_collisionRadius;
	DOUBLE m_collisionRadiusSquared;
	DOUBLE m_mass;
	Matrix33 m_inertiaTensor;
};