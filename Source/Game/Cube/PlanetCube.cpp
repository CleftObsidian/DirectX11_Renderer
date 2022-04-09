#include "Cube/PlanetCube.h"

void PlanetCube::Update(_In_ FLOAT deltaTime)
{
	// Rotate around origin
	XMMATRIX spin = XMMatrixRotationZ(-deltaTime);
	XMMATRIX orbit = XMMatrixRotationY(-deltaTime * 2.0f);
	XMMATRIX translate = XMMatrixTranslation(-4.0f, 0.0f, 0.0f);
	XMMATRIX scale = XMMatrixScaling(0.3f, 0.3f, 0.3f);

	m_world = scale * spin * translate * orbit;
}