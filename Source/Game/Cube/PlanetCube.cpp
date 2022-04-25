#include "Cube/PlanetCube.h"

/*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
  Method:   PlanetCube::PlanetCube

  Summary:  Constructor

  Args:     const std::filesystem::path& textureFilePath
			  Path to the texture to use
M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
PlanetCube::PlanetCube(const std::filesystem::path& textureFilePath)
	: BaseCube(textureFilePath)
{
	// empty
}

/*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
  Method:   PlanetCube::Update

  Summary:  Updates the cube every frame

  Args:     FLOAT deltaTime
			  Elapsed time

  Modifies: [m_world].
M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
void PlanetCube::Update(_In_ FLOAT deltaTime)
{
	// Add time
	static FLOAT s_totalTime = 0.0f;
	s_totalTime += deltaTime;

	// Rotate around origin
	static XMMATRIX s_spin = XMMatrixIdentity();
	s_spin = XMMatrixRotationZ(-s_totalTime);

	static XMMATRIX s_orbit = XMMatrixIdentity();
	s_orbit = XMMatrixRotationY(-s_totalTime * 2.0f);

	static XMMATRIX s_translate = XMMatrixIdentity();
	s_translate = XMMatrixTranslation(-4.0f, XMScalarSin(s_totalTime), 0.0f);

	static XMMATRIX s_scale = XMMatrixScaling(0.5f, 0.5f, 0.5f);

	m_world = s_scale * s_spin * s_translate * s_orbit;
}