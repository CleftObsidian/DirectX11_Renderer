#include "Texture/Material.h"

namespace library
{
    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Material::Material

      Summary:  Constructor

      Modifies: [pDiffuse, pSpecular].
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    /*--------------------------------------------------------------------
      TODO: Material::Material definition (remove the comment)
    --------------------------------------------------------------------*/
    Material::Material()
        : pDiffuse(std::shared_ptr<Texture>())
        , pSpecular(std::shared_ptr<Texture>())
    {
        // empty
    }
}