#include "Cube/PlanetCube.h"

PlanetCube::PlanetCube(const std::filesystem::path& textureFilePath)
	: BaseCube(textureFilePath)
	, m_spin()
	, m_orbit()
	, m_translate()
	, m_scale(XMMatrixScaling(0.5f, 0.5f, 0.5f))
{
	// empty
}

void PlanetCube::Update(_In_ FLOAT deltaTime)
{
	// Add time
	static FLOAT s_totalTime = 0.0f;
	s_totalTime += deltaTime;

	// Rotate around origin
	m_spin = XMMatrixRotationZ(-s_totalTime);
	m_orbit = XMMatrixRotationY(-s_totalTime * 2.0f);
	m_translate = XMMatrixTranslation(-4.0f, XMScalarSin(s_totalTime), 0.0f);

	m_world = m_scale * m_spin * m_translate * m_orbit;
}