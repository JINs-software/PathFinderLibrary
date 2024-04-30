// PathFinderSimulator.cpp : 애플리케이션에 대한 진입점을 정의합니다.
//
#include "PathFinderSimulator.h"
#include "PathFinderSimulatorConfig.h"

// 이 코드 모듈에 포함된 함수의 선언을 전달합니다:;
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // TODO: 여기에 코드를 입력합니다.

    // 전역 문자열을 초기화합니다.
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_PATHFINDERSIMULATOR, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // 애플리케이션 초기화를 수행합니다:
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_PATHFINDERSIMULATOR));

    MSG msg;

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int) msg.wParam;
}

//
//  함수: MyRegisterClass()
//
//  용도: 창 클래스를 등록합니다.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_PATHFINDERSIMULATOR));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_PATHFINDERSIMULATOR);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   함수: InitInstance(HINSTANCE, int)
//
//   용도: 인스턴스 핸들을 저장하고 주 창을 만듭니다.
//
//   주석:
//
//        이 함수를 통해 인스턴스 핸들을 전역 변수에 저장하고
//        주 프로그램 창을 만든 다음 표시합니다.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   /*** my init ***/
#if defined(GRID_RANGE_SETTING)
   RECT rect;
   GetClientRect(hWnd, &rect);
   int initCellSize = 20;
   int cY = GRID_RANGE_Y;
   int cX = GRID_RANGE_X;
   grid.SetGridCell(initCellSize, cY, cX);
   g_AStarPathFinder.Init(cY, cX);
   jpsFinder.Init(cY, cX);
#else
   RECT rect;
   GetClientRect(hWnd, &rect);
   int initCellSize = 20;
   int cY = rect.bottom / initCellSize;
   int cX = rect.right / initCellSize;
   grid.SetGridCell(initCellSize, cY, cX);
   pathFinder.Init(cY, cX);
   jpsFinder.Init(cY, cX);
#endif

   /****************/

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  함수: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  용도: 주 창의 메시지를 처리합니다.
//
//  WM_COMMAND  - 애플리케이션 메뉴를 처리합니다.
//  WM_PAINT    - 주 창을 그립니다.
//  WM_DESTROY  - 종료 메시지를 게시하고 반환합니다.
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_CREATE:
    {
        HDC hdc = GetDC(hWnd);
        GetClientRect(hWnd, &g_MemDC_Rect);
        g_hMemDC_Bitmap = CreateCompatibleBitmap(hdc, g_MemDC_Rect.right, g_MemDC_Rect.bottom);
        g_hMemDC = CreateCompatibleDC(hdc);
        ReleaseDC(hWnd, hdc);
        g_hMemDC_BitmapOld = (HBITMAP)SelectObject(g_hMemDC, g_hMemDC_Bitmap);

        // 콘솔 창 할당
        AllocConsole();
        AttachConsole(GetCurrentProcessId());
        freopen("CONOUT$", "w", stdout); // 표준 출력을 콘솔에 연결
        break;
    }
    case WM_COMMAND:
    {
        int wmId = LOWORD(wParam);
        // 메뉴 선택을 구문 분석합니다:
        switch (wmId)
        {
        case IDM_ABOUT:
            DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
            break;
        case IDM_EXIT:
            DestroyWindow(hWnd);
            break;
        case IDM_CLEAR:
            All_Clear();
            InvalidateRect(hWnd, NULL, TRUE);
            break;
        case IDM_ASTAR:
            astarFlag = true;
            InvalidateRect(hWnd, NULL, TRUE);
            break;
        case IDM_JPS:
            astarFlag = false;
            InvalidateRect(hWnd, NULL, TRUE);
            break;
        case IDM_MANHATTAN:
            //astar.SetHeuristicCalcAlgo(MANHATTAN);
            break;
        case IDM_EUCLIDEAN:
            //astar.SetHeuristicCalcAlgo(EUCLIDEAN);
            break;
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
        }
    }
    break;
    case WM_KEYDOWN:
    {
        switch (wParam) {
        case 'W':
            grid.MoveUp();
            break;
            grid.MoveUp();
        case 'w':
            break;
        case 'A':
            grid.MoveLeft();
            break;
        case 'a':
            grid.MoveLeft();
            break;
        case 'S':
            grid.MoveDown();
            break;
        case 's':
            grid.MoveDown();
            break;
        case 'D':
            grid.MoveRight();
            break;
        case 'd':
            grid.MoveRight();
            break;
        case VK_SPACE:
            if (procFlag) {
                loopProcess = !loopProcess;
            }
            else {
                if (startFlag && destFlag) {
                    /*
                    * 장애물이 셋팅되었음에도 grid에 셋팅되지 않은 오류들이 존재해 보임, 임시 방변으로 path finder에 보내기 전 한번 더 체크하는 로직 추가
                    */
                    //grid.cells;
                    for (int y = 0; y < grid.cells.size(); y++) {
                        for (int x = 0; x < grid.cells[0].size(); x++) {
                            if (grid.cells[y][x].GetEnColor() == BLACK) {
                                jpsFinder.SetObstacle(y, x);
                            }
                            else {
                                jpsFinder.UnsetObstacle(y, x);
                            }
                        }
                    }

                    trackList.clear();
                    if (astarFlag) {
                        path = g_AStarPathFinder.FindPath(startY, startX, destY, destX, trackList);
                        std::thread background(ProcessPaint, hWnd);
                        background.detach();
                    }
                    else {
                        pathJps = jpsFinder.FindPath(startY, startX, destY, destX, trackList);
                        std::thread background(ProcessPaintJPS, hWnd);
                        background.detach();
                    }
                }
                else {
                    // 경고창 띄우기
                    MessageBox(NULL, L"출발지/목적지 지정 필요", L"경고", MB_OK | MB_ICONWARNING);
                }
            }
            break;
        case VK_ESCAPE:
            grid.ClearPath();
            break;
        }
        // 윈도우를 다시 그리도록 강제
        InvalidateRect(hWnd, NULL, TRUE);
        break;

    }
    case WM_MOUSEWHEEL: {
        POINT cursorPos;
        GetCursorPos(&cursorPos);
        ScreenToClient(hWnd, &cursorPos);
        
        int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        // 휠을 위로 굴리면 격자 확대
        if (delta > 0) { grid.cellSize = 100 < grid.cellSize + 5 ? 100 : grid.cellSize + 5; }
        // 휠을 아래로 굴리면 격자 축소 (최소 크기는 5)
        else { grid.cellSize = 5 > grid.cellSize - 5 ? 5 : grid.cellSize - 5; }

        // 윈도우를 다시 그리도록 강제
        InvalidateRect(hWnd, NULL, TRUE);
        break;
    }
    case WM_MOUSEMOVE:
    {
        // 마우스 왼쪽 버튼을 누른 상태에서 이동하면 격자를 이동
        if (wParam & MK_MBUTTON) {
            int mouseX = LOWORD(lParam);
            int mouseY = HIWORD(lParam);

            if (mouseX < prevMouseX) {
                grid.MoveLeft();
            }
            else if (mouseX > prevMouseX) {
                grid.MoveRight();
            }
            if (mouseY < prevMouseY) {
                grid.MoveUp();
            }
            else if (mouseY > prevMouseY) {
                grid.MoveDown();
            }

            // 윈도우를 다시 그리도록 강제
            InvalidateRect(hWnd, NULL, TRUE);

            prevMouseX = mouseX;
            prevMouseY = mouseY;
        }
        else if (wParam & MK_LBUTTON) {
            if (colorIndex == BLACK) {
                pair<int, int> YX = grid.GetCellPos(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam));
                if (YX.first == -1 || YX.second == -1) {
                    break;
                }
                if (lbtnDownFlag && (focusCellY == YX.first && focusCellX == YX.second)) {
                    break;
                }
                else {
                    focusCellY = YX.first;
                    focusCellX = YX.second;
                }
                 
                enColor cellColor = grid.cells[YX.first][YX.second].GetEnColor();
                if (cellColor == RED || cellColor == BLUE) {
                    break;
                }
                else if (cellColor == WHITE_DEFAULT) {
                    if (grid.HandleLeftClick(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam), BLACK)) {
                        //astar.SetObstacle(YX.first, YX.second);
                        g_AStarPathFinder.SetObstacle(YX.first, YX.second);
                        jpsFinder.SetObstacle(YX.first, YX.second);
                    }
                }
                else if (cellColor == BLACK) {
                    if (grid.HandleLeftClick(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam), WHITE_DEFAULT)) {
                        //astar.UnsetObstacle(YX.first, YX.second);
                        g_AStarPathFinder.UnsetObstacle(YX.first, YX.second);
                        jpsFinder.UnsetObstacle(YX.first, YX.second);
                    }
                }

                // 윈도우를 다시 그리도록 강제
                InvalidateRect(hWnd, NULL, TRUE);
            }
        }
        break;
    }
    case WM_LBUTTONDOWN:
    {
        bool chgFlag = false;
        pair<int, int> YX = grid.GetCellPos(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam));
        if (YX.first == -1 || YX.second == -1) {
            break;
        }
        enColor cellColor = grid.cells[YX.first][YX.second].GetEnColor();
        // RED 모드 클릭
        if (colorIndex == RED) {
            if (cellColor == BLUE || cellColor == BLACK) {
                break;
            }
            else if (cellColor == RED) {
                grid.HandleLeftClick(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam), WHITE_DEFAULT);
                startFlag = false;
                chgFlag = true;
            }
        }
        // BLUE 클릭
        else if (colorIndex == BLUE) { //&& blueFlag) {
            if (cellColor == RED || cellColor == BLACK) {
                break;
            }
            else if (cellColor == BLUE) {
                grid.HandleLeftClick(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam), WHITE_DEFAULT);
                destFlag = false;
                chgFlag = true;
            }
        }
        // BLACK 클릭
        else {
            lbtnDownFlag = true;
            focusCellY = YX.first;
            focusCellX = YX.second;

            if (cellColor == RED || cellColor == BLUE) {
                break;
            }
            else if (cellColor == BLACK) {
                grid.HandleLeftClick(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam), WHITE_DEFAULT);
                g_AStarPathFinder.UnsetObstacle(YX.first, YX.second);
                jpsFinder.UnsetObstacle(YX.first, YX.second);
                chgFlag = true;
            }
        }

        if (!chgFlag) {
            if (grid.HandleLeftClick(GET_Y_LPARAM(lParam), GET_X_LPARAM(lParam), colorIndex)) {
                if (colorIndex == RED) {
                    startY = YX.first;
                    startX = YX.second;
                    startFlag = true;
                }
                else if (colorIndex == BLUE) {
                    //astar.SetDestPos(YX.first, YX.second);
                    destY = YX.first;
                    destX = YX.second;
                    destFlag = true;
                }
                else if (colorIndex == BLACK) {
                    g_AStarPathFinder.SetObstacle(YX.first, YX.second);
                    jpsFinder.SetObstacle(YX.first, YX.second);
                }
            }
        }

        // 윈도우를 다시 그리도록 강제
        InvalidateRect(hWnd, NULL, TRUE);
        break;
    }
    case WM_LBUTTONUP:
    {
        lbtnDownFlag = false;
        break;
    }
    case WM_RBUTTONDOWN:
    {
        // 마우스 우클릭 이벤트 처리
            // R, G, B 성분을 순환하며 커서 색상 변경
        colorIndex = static_cast<enColor>((static_cast<int>(colorIndex) + 1) % enColor::COLOR_CNT);

        // 윈도우를 다시 그리도록 강제
        InvalidateRect(hWnd, NULL, TRUE);
        break;
    }
    case WM_DESTROY:
        SelectObject(g_hMemDC, g_hMemDC_BitmapOld);
        DeleteObject(g_hMemDC);
        DeleteObject(g_hMemDC_Bitmap);
        PostQuitMessage(0);
        break;
    case WM_PAINT:
    {
        drawFlag = true;
        PAINTSTRUCT ps;

        // 메모리 DC 클리어
        PatBlt(g_hMemDC, 0, 0, g_MemDC_Rect.right, g_MemDC_Rect.bottom, WHITENESS);

        // 메모리 DC에 출력
        grid.Draw(g_hMemDC, g_MemDC_Rect.right, g_MemDC_Rect.bottom);
        DrawInformationBox(g_hMemDC);
        DrawInformationText(g_hMemDC, colorIndex, startX, startY, destX, destY);

        //  메모리 DC에 출력이 끝나면, 메모리 DC -> 윈도우 DC로 출력
        HDC hdc = BeginPaint(hWnd, &ps);
        BitBlt(hdc, 0, 0, g_MemDC_Rect.right, g_MemDC_Rect.bottom, g_hMemDC, 0, 0, SRCCOPY);
        EndPaint(hWnd, &ps);

        drawFlag = false;
        break;
    }
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// 정보 대화 상자의 메시지 처리기입니다.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}



////////////////////////////////////////////////////////
void All_Clear() {
    startFlag = false;
    destFlag = false;
    grid.Clear();
}
void DrawInformationBox(HDC hdc) {
    // 반투명 박스 그리기 (좌상단에 위치하도록 설정)
    BLENDFUNCTION blend = { AC_SRC_OVER, 0, static_cast<BYTE>(235), AC_SRC_OVER };
    COLORREF colorKey = RGB(0xa9, 0xa9, 0xa9);

    // 박스 크기 조절
    int boxWidth = 435;
    int boxHeight = 375;

    HDC memDC = CreateCompatibleDC(hdc);
    HBITMAP memBitmap = CreateCompatibleBitmap(hdc, boxWidth, boxHeight); // 원하는 크기로 설정
    HBITMAP oldBitmap = (HBITMAP)SelectObject(memDC, memBitmap);

    // 메모리 DC에 원하는 그림 그리기
    HBRUSH hBrush = CreateSolidBrush(colorKey);
    RECT upleftRect{ 0, 0, boxWidth, boxHeight };
    FillRect(memDC, &upleftRect, hBrush);
    DeleteObject(hBrush);

    // 메모리 DC의 내용을 윈도우에 그리기
    BOOL result = AlphaBlend(hdc, 0, 0, boxWidth, boxHeight, memDC, 0, 0, boxWidth, boxHeight, blend);

    // 해제
    SelectObject(memDC, oldBitmap);
    DeleteObject(memBitmap);
    DeleteDC(memDC);
}

void DrawInformationText(HDC hdc, enColor colorMode, PosType startX, PosType startY, PosType destX, PosType destY) {
    std::wstring startXstr;
    std::wstring startYstr;
    std::wstring destXstr;
    std::wstring destYstr;
    
    if (startFlag) {
        startXstr = std::to_wstring(startX);
        startYstr = std::to_wstring(startY);
    }
    else {
        startXstr = L"none";
        startYstr = L"none";
    }
    if (destFlag) {
        destXstr = std::to_wstring(destX);
        destYstr = std::to_wstring(destY);
    }
    else {
        destXstr = L"none";
        destYstr = L"none";
    }
    
    std::wstring caption;
    if (astarFlag) {
        caption = L"A* 알고리즘";
    }
    else {
        caption = L"JPS 알고리즘";
    }

    std::wstring usage = L"[usage]";
    std::vector<wstring> usageVec = {
        L"·격자 이동: W(상) / S(하) / A(좌) / D(우)",
        L"·격자 확대/축소: 마우스 휠 up/down",
        L"·마우스 입력 모드 변경: 마우스 우 클릭",
        L"                                           - red: 출발지 지정 모드",
        L"                                           - blue: 도착지 지정 모드",
        L"                                           - black: 장애물|벽 지정 모드",
        L"·출발지 지정: 마우스 좌 클릭 + 출발지 지정 모드",
        L"·도착지 지정: 마우스 좌 클릭 + 도착지 지정 모드",
        L"·장애물|벽 지정: 마우스 좌 클릭 + 장애물/벽 지정 모드"
    };

    std::wstring state = L"[state]";
    std::wstring mode = L"·마우스 입력 모드: ";
    std::wstring modeColor;
    switch (colorMode) {
    case RED:
        modeColor = L"출발지 지정";
        break;
    case BLUE:
        modeColor = L"도착지 지정";
        break;
    case BLACK:
        modeColor = L"장애물|벽 지정";
        break;
    }
    std::wstring startPos = L"·출발지 X: " + startXstr + L", Y: " + startYstr;
    std::wstring destPos = L"·도착치 X: " + destXstr + L", Y: " + destYstr;

    // 폰트 생성
    HFONT hFont = CreateFont(30, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_OUTLINE_PRECIS,
        CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Verdana");
    // 검은색 펜 생성
    HPEN hPen = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
    // 폰트 및 펜 선택
    SelectObject(hdc, hFont);
    SelectObject(hdc, hPen);
    // 배경을 투명으로 설정
    SetBkMode(hdc, TRANSPARENT);
    // 검은색 글자 출력
    SetTextColor(hdc, RGB(0, 0, 0));
    TextOut(hdc, 10, 10, caption.c_str(), caption.length());

    // 폰트 크기를 변경해서 아래 TextOut 출력
    HFONT hFontSmall = CreateFont(20, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_OUTLINE_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Verdana");
    // [usage]
    SelectObject(hdc, hFontSmall);
    TextOut(hdc, 10, 50, usage.c_str(), usage.length());
    TextOut(hdc, 10, 70, usageVec[0].c_str(), usageVec[0].length());
    TextOut(hdc, 10, 90, usageVec[1].c_str(), usageVec[1].length());
    TextOut(hdc, 10, 110, usageVec[2].c_str(), usageVec[2].length());
    hFontSmall = CreateFont(15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_OUTLINE_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Verdana");
    SelectObject(hdc, hFontSmall);
    SetTextColor(hdc, RGB(255, 0, 0));
    TextOut(hdc, 10, 130, usageVec[3].c_str(), usageVec[3].length());
    SetTextColor(hdc, RGB(0, 0, 255));
    TextOut(hdc, 10, 150, usageVec[4].c_str(), usageVec[4].length());
    SetTextColor(hdc, RGB(0, 0, 0));
    TextOut(hdc, 10, 170, usageVec[5].c_str(), usageVec[5].length());
    SetTextColor(hdc, RGB(0, 0, 0));
    hFontSmall = CreateFont(20, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_OUTLINE_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Verdana");
    SelectObject(hdc, hFontSmall);
    TextOut(hdc, 10, 210, usageVec[6].c_str(), usageVec[6].length());
    TextOut(hdc, 10, 230, usageVec[7].c_str(), usageVec[7].length());
    TextOut(hdc, 10, 250, usageVec[8].c_str(), usageVec[8].length());

    // [state]
    TextOut(hdc, 10, 280, state.c_str(), state.length());
    TextOut(hdc, 10, 300, mode.c_str(), mode.length());
    switch (colorMode) {
    case RED:
        SetTextColor(hdc, RGB(255, 0, 0));
        break;
    case BLUE:
        SetTextColor(hdc, RGB(0, 0, 255));
        break;
    case BLACK:
        SetTextColor(hdc, RGB(0, 0, 0));
        break;
    }
    hFontSmall = CreateFont(20, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_OUTLINE_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Verdana");
    SelectObject(hdc, hFontSmall);
    TextOut(hdc, 145, 300, modeColor.c_str(), modeColor.length());
    SetTextColor(hdc, RGB(0, 0, 0));
    hFontSmall = CreateFont(20, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_OUTLINE_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Verdana");
    SelectObject(hdc, hFontSmall);

    TextOut(hdc, 10, 320, startPos.c_str(), startPos.length());
    TextOut(hdc, 10, 340, destPos.c_str(), destPos.length());
    DeleteObject(hFontSmall);

    // 폰트 및 펜 해제
    DeleteObject(hFont);
    DeleteObject(hPen);
}

/*********************/
/*** My Func ***/
//void PaintMiddleCell(HWND hWnd) {
//    HDC hMemDC;
//    HBITMAP hMemDC_Bitmap;
//    HBITMAP hMemDC_BitmapOld;
//    RECT MemDC_Rect;
//
//    HDC hdc = GetDC(hWnd);
//    GetClientRect(hWnd, &MemDC_Rect);
//    hMemDC_Bitmap = CreateCompatibleBitmap(hdc, MemDC_Rect.right, MemDC_Rect.bottom);
//    hMemDC = CreateCompatibleDC(hdc);
//    hMemDC_BitmapOld = (HBITMAP)SelectObject(hMemDC, hMemDC_Bitmap);
//
//    PatBlt(hMemDC, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, WHITENESS);
//
//    while (middleFlag) {
//        mqueueLock.lock();
//        while (!middleNodeQueue.empty()) {
//            PathNode<PosType>& node = middleNodeQueue.front();
//            middleNodeQueue.pop();
//            Cell& cell = grid.cells[node.pos.y][node.pos.x];
//            enColor celEncolor = cell.GetEnColor();
//            cell.G = to_wstring(node.g);
//            cell.H = to_wstring(node.h);
//            cell.F = to_wstring(node.f);
//            if (celEncolor != RED && celEncolor != BLUE) {
//                cell.SetColor(BLUE);
//            }
//            
//            grid.Draw(hMemDC, MemDC_Rect.right, MemDC_Rect.bottom);
//            DrawInformationBox(hMemDC);
//            DrawInformationText(hMemDC, L"A* 알고리즘", colorIndex, startX, startY, destX, destY);
//
//            BitBlt(hdc, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, hMemDC, 0, 0, SRCCOPY);
//        }
//        mqueueLock.unlock();
//    }
//
//    ReleaseDC(hWnd, hdc);
//
//    SelectObject(hMemDC, hMemDC_BitmapOld);
//    DeleteObject(hMemDC_Bitmap);
//    DeleteObject(hMemDC);
//}
/***************/