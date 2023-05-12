#include "Game/Game.h"
#include "Math/Transformation3D.h"

PhysicsObject* createBox(DOUBLE x, DOUBLE y, DOUBLE z, DOUBLE xPos, DOUBLE yPos, DOUBLE zPos, BOOL fixed)
{
    Vector3D p1 = Vector3D(-x / 2.0, -y / 2.0, -z / 2.0);
    Vector3D p2 = Vector3D(-x / 2.0, y / 2.0, -z / 2.0);
    Vector3D p3 = Vector3D(x / 2.0, -y / 2.0, -z / 2.0);
    Vector3D p4 = Vector3D(x / 2.0, y / 2.0, -z / 2.0);
    Vector3D p5 = Vector3D(-x / 2.0, -y / 2.0, z / 2.0);
    Vector3D p6 = Vector3D(-x / 2.0, y / 2.0, z / 2.0);
    Vector3D p7 = Vector3D(x / 2.0, -y / 2.0, z / 2.0);
    Vector3D p8 = Vector3D(x / 2.0, y / 2.0, z / 2.0);

    Vector3D translation(xPos, yPos, zPos);
    std::vector<Vector3D*> points;
    points.push_back(&p1);
    points.push_back(&p2);
    points.push_back(&p3);
    points.push_back(&p4);
    points.push_back(&p5);
    points.push_back(&p6);
    points.push_back(&p7);
    points.push_back(&p8);
    Transformation3D::TranslatePoints(&points, translation);

    std::vector<RigidSurface*> surfaces;

    std::vector<Vector3D> s1Points;
    s1Points.push_back(p1);
    s1Points.push_back(p3);
    s1Points.push_back(p7);
    s1Points.push_back(p5);
    surfaces.push_back(new RigidSurface(&s1Points, Vector3D(0, -1, 0)));

    std::vector<Vector3D> s2Points;
    s2Points.push_back(p2);
    s2Points.push_back(p4);
    s2Points.push_back(p8);
    s2Points.push_back(p6);
    surfaces.push_back(new RigidSurface(&s2Points, Vector3D(0, 1, 0)));

    std::vector<Vector3D> s3Points;
    s3Points.push_back(p1);
    s3Points.push_back(p2);
    s3Points.push_back(p6);
    s3Points.push_back(p5);
    surfaces.push_back(new RigidSurface(&s3Points, Vector3D(-1, 0, 0)));

    std::vector<Vector3D> s4Points;
    s4Points.push_back(p3);
    s4Points.push_back(p4);
    s4Points.push_back(p8);
    s4Points.push_back(p7);
    surfaces.push_back(new RigidSurface(&s4Points, Vector3D(1, 0, 0)));

    std::vector<Vector3D> s5Points;
    s5Points.push_back(p1);
    s5Points.push_back(p2);
    s5Points.push_back(p4);
    s5Points.push_back(p3);
    surfaces.push_back(new RigidSurface(&s5Points, Vector3D(0, 0, -1)));

    std::vector<Vector3D> s6Points;
    s6Points.push_back(p5);
    s6Points.push_back(p6);
    s6Points.push_back(p8);
    s6Points.push_back(p7);
    surfaces.push_back(new RigidSurface(&s6Points, Vector3D(0, 0, 1)));

    std::vector<ConvexHull*> hulls;
    hulls.push_back(new ConvexHull(&surfaces, 1));
    RigidBody* body = new RigidBody(hulls, 1, 1, 0.3, fixed);
    return new PhysicsObject(body);
}

namespace library
{
    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Game::Game

      Summary:  Constructor

      Args:     PCWSTR pszGameName
                  Name of the game

      Modifies: [m_pszGameName, m_mainWindow, m_renderer].
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    Game::Game(_In_ PCWSTR pszGameName)
        : m_pszGameName(pszGameName)
        , m_mainWindow(std::make_unique<MainWindow>())
        , m_renderer(std::make_unique<Renderer>())
        , m_physicsEngine(std::make_unique<PhysicsEngine>(0.00167))
        , m_objects()
    {
        // empty
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Game::Initialize

      Summary:  Initializes the components of the game

      Args:     HINSTANCE hInstance
                  Handle to the instance
              INT nCmdShow
                  Is a flag that says whether the main application window
                  will be minimized, maximized, or shown normally

      Modifies: [m_mainWindow, m_renderer].

      Returns:  HRESULT
                  Status code
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    HRESULT Game::Initialize(_In_ HINSTANCE hInstance, _In_ INT	nCmdShow)
    {
        if (FAILED(m_mainWindow->Initialize(hInstance, nCmdShow, m_pszGameName)))
        {
            return E_FAIL;
        }

        if (FAILED(m_renderer->Initialize(m_mainWindow->GetWindow())))
        {
            return E_FAIL;
        }

        return S_OK;
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Game::Run

      Summary:  Runs the game loop

      Returns:  INT
                  Status code to return to the operating system
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    INT Game::Run()
    {
        // Initialize time
        LARGE_INTEGER LastTime;
        LARGE_INTEGER CurrentTime;
        LARGE_INTEGER Frequency;

        FLOAT deltaTime;

        QueryPerformanceFrequency(&Frequency);
        QueryPerformanceCounter(&LastTime);

        // Main message loop
        MSG msg = { 0 };
        while (WM_QUIT != msg.message)
        {
            if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
            {
                // Call WndProc Function
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
            else
            {
                // Update our time
                QueryPerformanceCounter(&CurrentTime);

                deltaTime = static_cast<FLOAT>(CurrentTime.QuadPart - LastTime.QuadPart);
                deltaTime /= static_cast<FLOAT>(Frequency.QuadPart);

                LastTime = CurrentTime;

                // Handle input
                m_renderer->HandleInput(m_mainWindow->GetDirections(), m_mainWindow->GetMouseRelativeMovement(), deltaTime);
                m_mainWindow->ResetMouseMovement();

                // Render
                m_renderer->Update(deltaTime);
                m_renderer->Render();
            }
        }

        return static_cast<INT>(msg.wParam);
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Game::GetGameName

      Summary:  Returns the name of the game

      Returns:  PCWSTR
                  Name of the game
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    PCWSTR Game::GetGameName() const
    {
        return m_pszGameName;
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Game::GetWindow

      Summary:  Returns the main window

      Returns:  std::unique_ptr<MainWindow>&
                  The main window
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    std::unique_ptr<MainWindow>& Game::GetWindow()
    {
        return m_mainWindow;
    }

    /*M+M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M+++M
      Method:   Game::GetRenderer

      Summary:  Returns the renderer

      Returns:  std::unique_ptr<Renderer>&
                  The renderer
    M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M---M-M*/
    std::unique_ptr<Renderer>& Game::GetRenderer()
    {
        return m_renderer;
    }
}