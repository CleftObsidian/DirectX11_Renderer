#include "OctreeNode.h"

OctreeNode::OctreeNode(Vector3D pos, DOUBLE size, DOUBLE minSize)
	: m_pos(pos)
	, m_size(size)
	, m_minSize(minSize)
	, m_bIsLeafNode()
	, m_apBodies()
	, m_apChildren{}
{
	for (UINT index = 0; index < N_CHILD; ++index)
	{
		m_apChildren[index] = nullptr;
	}
}

OctreeNode::~OctreeNode()
{
	if (m_bIsLeafNode)
	{
		return;
	}

	for (UINT index = 0; index < N_CHILD; ++index)
	{
		if (m_apChildren[index] != nullptr)
		{
			delete m_apChildren[index];
		}
	}
}

DOUBLE OctreeNode::GetSize()
{
	return m_size;
}

Vector3D OctreeNode::GetPos()
{
	return m_pos;
}

void OctreeNode::ExpandNode()
{
	if (m_size <= m_minSize || m_apBodies.size() <= 1)
	{
		m_bIsLeafNode = TRUE;
		return;	//no more children, end tree
	}

	for (RigidBody* body : m_apBodies)
	{
		Vector3D bPos = body->GetCenterOfMass();
		Vector3D dim;
		if (body->GetIsFixed())
		{
			bPos = body->GetDimCenter();
			dim = body->GetDimensions().ScalarMultiply(0.5);
		}
		else
		{
			DOUBLE r = body->GetCollisionRadius();
			dim = Vector3D(r, r, r);
		}

		// lack of else statements intentional,
		// as bodies can be in multiple nodes at once
		// e.g. sitting on edge

		//posX
		if (bPos.m_x + dim.m_x > m_pos.m_x + m_size / 2.0)
		{
			//posZ
			if (bPos.m_z + dim.m_z > m_pos.m_z + m_size / 2.0)
			{
				//posY
				if (bPos.m_y + dim.m_y > m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[0] == nullptr)
					{
						m_apChildren[0] = new OctreeNode(m_pos.Add(Vector3D(m_size / 2.0, m_size / 2.0, m_size / 2.0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[0]->AddBody(body);
				}
				//negY
				if (bPos.m_y - dim.m_y < m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[1] == nullptr)
					{
						m_apChildren[1] = new OctreeNode(m_pos.Add(Vector3D(m_size / 2.0, 0, m_size / 2.0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[1]->AddBody(body);
				}
			}
			//negZ
			if (bPos.m_z - dim.m_z < m_pos.m_z + m_size / 2.0)
			{
				//posY
				if (bPos.m_y + dim.m_y > m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[2] == nullptr)
					{
						m_apChildren[2] = new OctreeNode(m_pos.Add(Vector3D(m_size / 2.0, m_size / 2.0, 0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[2]->AddBody(body);
				}
				//negY
				if (bPos.m_y - dim.m_y < m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[3] == nullptr)
					{
						m_apChildren[3] = new OctreeNode(m_pos.Add(Vector3D(m_size / 2.0, 0, 0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[3]->AddBody(body);
				}
			}
		}
		//negX
		if (bPos.m_x - dim.m_x < m_pos.m_x + m_size / 2.0)
		{
			//posZ
			if (bPos.m_z + dim.m_z > m_pos.m_z + m_size / 2.0)
			{
				//posY
				if (bPos.m_y + dim.m_y > m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[4] == nullptr)
					{
						m_apChildren[4] = new OctreeNode(m_pos.Add(Vector3D(0, m_size / 2.0, m_size / 2.0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[4]->AddBody(body);
				}
				//negY
				if (bPos.m_y - dim.m_y < m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[5] == nullptr)
					{
						m_apChildren[5] = new OctreeNode(m_pos.Add(Vector3D(0, 0, m_size / 2.0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[5]->AddBody(body);
				}
			}
			//negZ
			if (bPos.m_z - dim.m_z < m_pos.m_z + m_size / 2.0)
			{
				//posY
				if (bPos.m_y + dim.m_y > m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[6] == nullptr)
					{
						m_apChildren[6] = new OctreeNode(m_pos.Add(Vector3D(0, m_size / 2.0, 0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[6]->AddBody(body);
				}
				//negY
				if (bPos.m_y - dim.m_y < m_pos.m_y + m_size / 2.0)
				{
					if (m_apChildren[7] == nullptr)
					{
						m_apChildren[7] = new OctreeNode(m_pos.Add(Vector3D(0, 0, 0)), m_size / 2.0, m_minSize);
					}
					m_apChildren[7]->AddBody(body);
				}
			}
		}
	}

	m_bIsLeafNode = TRUE;
	for (UINT index = 0; index < N_CHILD; ++index)
	{
		if (m_apChildren[index] != nullptr)
		{
			m_bIsLeafNode = FALSE;
			m_apChildren[index]->ExpandNode();
		}
	}
}

void OctreeNode::AddBody(RigidBody* body)
{
	m_apBodies.push_back(body);
}

BOOL OctreeNode::IsLeafNode()
{
	return m_bIsLeafNode;
}

std::vector<RigidBody*>* OctreeNode::GetBodies()
{
	return &m_apBodies;
}

void OctreeNode::GetCollisionLeafs(std::vector<OctreeNode*>* out)
{
	getCollisionLeafs(this, out);
}

void OctreeNode::GetAllNodes(std::vector<OctreeNode*>* out)
{
	getAllNodes(this, out);
}

void OctreeNode::getCollisionLeafs(OctreeNode* curr, std::vector<OctreeNode*>* out)
{
	if (curr->IsLeafNode() && (curr->m_apBodies.size() >= 2))
	{
		out->push_back(curr);
		return;
	}
	else if (!curr->IsLeafNode())
	{
		for (int index = 0; index < N_CHILD; ++index)
		{
			if (curr->m_apChildren[index] != nullptr)
			{
				getCollisionLeafs(curr->m_apChildren[index], out);
			}
		}
	}
}

void OctreeNode::getAllNodes(OctreeNode* curr, std::vector<OctreeNode*>* out)
{
	out->push_back(curr);
	if (curr->IsLeafNode() && (curr->m_apBodies.size() >= 2))
	{
		return;
	}
	else if (!curr->IsLeafNode())
	{
		for (int index = 0; index < N_CHILD; ++index)
		{
			if (curr->m_apChildren[index] != nullptr)
			{
				getAllNodes(curr->m_apChildren[index], out);
			}
		}
	}
}
