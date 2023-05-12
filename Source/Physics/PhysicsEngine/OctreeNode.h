#pragma once

#include "RigidBody.h"

class OctreeNode
{
public:
	OctreeNode() = delete;
	OctreeNode(Vector3D pos, DOUBLE size, DOUBLE minSize);
	~OctreeNode();

	DOUBLE GetSize();
	Vector3D GetPos();
	void ExpandNode();
	void AddBody(RigidBody* body);
	BOOL IsLeafNode();
	std::vector<RigidBody*>* GetBodies();
	void GetCollisionLeafs(std::vector<OctreeNode*>* out);
	void GetAllNodes(std::vector<OctreeNode*>* out);

private:
	static void getCollisionLeafs(OctreeNode* curr, std::vector<OctreeNode*>* out);
	static void getAllNodes(OctreeNode* curr, std::vector<OctreeNode*>* out);	//for debug

private:
	static const INT N_CHILD = 8;

	Vector3D m_pos;
	DOUBLE m_size;
	DOUBLE m_minSize;
	BOOL m_bIsLeafNode;
	std::vector<RigidBody*> m_apBodies;
	OctreeNode* m_apChildren[N_CHILD];
};