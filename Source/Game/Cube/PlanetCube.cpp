#include "Cube/PlanetCube.h"

void PlanetCube::Update(_In_ FLOAT deltaTime)
{
	// Add time
	if (XMMatrixIsIdentity(m_world))
	{
		m_totalTime = 0.0f;
	}
	m_totalTime += deltaTime;

	// Rotate around origin
	XMMATRIX spin = XMMatrixRotationZ(-m_totalTime);
	XMMATRIX orbit = XMMatrixRotationY(-m_totalTime * 2.0f);
	XMMATRIX translate = XMMatrixTranslation(-4.0f, 0.0f, 0.0f);
	XMMATRIX scale = XMMatrixScaling(0.3f, 0.3f, 0.3f);

	m_world = scale * spin * translate * orbit;
}