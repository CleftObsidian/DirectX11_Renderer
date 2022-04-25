#include "Cube/SunCube.h"

/*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
  Method:   SunCube::SunCube

  Summary:  Constructor

  Args:     const std::filesystem::path& textureFilePath
			  Path to the texture to use
M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
SunCube::SunCube(const std::filesystem::path& textureFilePath)
	: BaseCube(textureFilePath)
{
	// empty
}

/*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
  Method:   SunCube::Update

  Summary:  Updates the cube every frame

  Args:     FLOAT deltaTime
			  Elapsed time

  Modifies: [m_world].
M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
void SunCube::Update(_In_ FLOAT deltaTime)
{
	// Does nothing
}