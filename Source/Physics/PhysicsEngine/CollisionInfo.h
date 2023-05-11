#pragma once

#include "Common.h"

class CollisionInfo
{
public:
	CollisionInfo() = delete;
	CollisionInfo(UINT16 collisionID, DOUBLE magnitude);
	~CollisionInfo();

private:
	UINT16 m_otherBodyID;
	DOUBLE m_collisionMagenitude;
};

inline CollisionInfo::CollisionInfo(UINT16 collisionID, DOUBLE magnitude)
	: m_otherBodyID(collisionID)
	, m_collisionMagenitude(magnitude)
{
}

inline CollisionInfo::~CollisionInfo()
{
}