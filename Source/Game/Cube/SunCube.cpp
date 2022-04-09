#include "Cube/SunCube.h"

void SunCube::Update(_In_ FLOAT deltaTime)
{
	// Rotate around the origin
	m_world = XMMatrixRotationY(deltaTime);
}