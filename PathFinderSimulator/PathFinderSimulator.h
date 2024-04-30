#pragma once

#include "resource.h"
#include "framework.h"

#pragma comment(lib, "Msimg32.lib")

/* my include */
#include <thread>
#include <queue>
#include <mutex>
#include "Grid.h"

#include <AStarPathFinder.h>
#include <JPSPathFinder.h>


#define MAX_LOADSTRING 100

#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))

// 메뉴 아이템 식별자
#define IDM_HEURISTIC_MENU  1001
#define IDM_OPTION1         101
#define IDM_OPTION2         102

#define PosType int
#define PosPoint std::pair<PosType, PosType>

// 전역 변수:
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.

/*** 메모리 DC 관련 변수들 ***/
HDC g_hMemDC;
HBITMAP g_hMemDC_Bitmap;
HBITMAP g_hMemDC_BitmapOld;
RECT g_MemDC_Rect;

bool drawFlag = false;

/*** My Global Val ***/
Grid grid;
//AStarAlgorithm astar;

PosType startX, startY;
bool startFlag = false;
PosType destX, destY;
bool destFlag = false;

bool astarFlag = true;
AStarPathFinder<PosType> g_AStarPathFinder;
PathFinder<PosType>::iterator path;

JPSPathFinder<PosType> jpsFinder;
PathFinder<PosType>::iterator pathJps;

std::vector<PathNode<PosType>> trackList;

PosType prevMouseX;
PosType prevMouseY;

enColor colorIndex;

PosType focusCellY;
PosType focusCellX;


bool lbtnDownFlag = false;
bool loopProcess = true;
bool procFlag = false;

std::queue<PathNode<PosType>> middleNodeQueue;
std::mutex mqueueLock;
bool middleFlag = false;


/*********************/
/*** My Func ***/
void All_Clear();
void DrawInformationBox(HDC hdc);
void DrawInformationText(HDC hdc, enColor colorMode, PosType startX, PosType startY, PosType destX, PosType destY);

void ProcessPaint(HWND hWnd) {
    HDC hMemDC;
    HBITMAP hMemDC_Bitmap;
    HBITMAP hMemDC_BitmapOld;
    RECT MemDC_Rect;

    HDC hdc = GetDC(hWnd);
    GetClientRect(hWnd, &MemDC_Rect);
    hMemDC_Bitmap = CreateCompatibleBitmap(hdc, MemDC_Rect.right, MemDC_Rect.bottom);
    hMemDC = CreateCompatibleDC(hdc);
    hMemDC_BitmapOld = (HBITMAP)SelectObject(hMemDC, hMemDC_Bitmap);

    PatBlt(hMemDC, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, WHITENESS);

    procFlag = true;
    //trackList = astar.GetTrackList();
    //optimalRoute = astar.GetOptimalRoute();

    for (PathNode<PosType>& node : trackList) {

        if (!loopProcess) {
            while (!loopProcess) {}
        }
        Cell& cell = grid.cells[node.pos.y][node.pos.x];
        enColor celEncolor = cell.GetEnColor();
        cell.G = to_wstring(node.g);
        cell.H = to_wstring(node.h);
        cell.F = to_wstring(node.f);
        if (celEncolor != RED && celEncolor != BLUE) {
            cell.SetColor(YELLOW);
        }

        grid.Draw(hMemDC, MemDC_Rect.right, MemDC_Rect.bottom);
        DrawInformationBox(hMemDC);
        DrawInformationText(hMemDC, colorIndex, startX, startY, destX, destY);
        BitBlt(hdc, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, hMemDC, 0, 0, SRCCOPY);

        while (drawFlag) {};

    }

    if (!path.End()) {
        while (!path.End()) {
            PathPosition<PosType> pp = *path;
            if (!loopProcess) {
                while (!loopProcess) {}
            }

            Cell& cell = grid.cells[pp.y][pp.x];
            enColor celEncolor = cell.GetEnColor();
            if (celEncolor != RED && celEncolor != BLUE) {
                cell.SetColor(GREEN);

                grid.Draw(hMemDC, MemDC_Rect.right, MemDC_Rect.bottom);
                DrawInformationBox(hMemDC);
                DrawInformationText(hMemDC, colorIndex, startX, startY, destX, destY);
                BitBlt(hdc, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, hMemDC, 0, 0, SRCCOPY);

                while (drawFlag) {};
            }

            ++path;
        }
    }
    else {
        MessageBox(NULL, L"목적지 도달 불가", L"경고", MB_OK | MB_ICONWARNING);
    }

    SelectObject(hMemDC, hMemDC_BitmapOld);
    DeleteObject(hMemDC_Bitmap);
    DeleteObject(hMemDC);

    procFlag = false;
}

void ProcessPaintJPS(HWND hWnd) {
    HDC hMemDC;
    HBITMAP hMemDC_Bitmap;
    HBITMAP hMemDC_BitmapOld;
    RECT MemDC_Rect;

    HDC hdc = GetDC(hWnd);
    GetClientRect(hWnd, &MemDC_Rect);
    hMemDC_Bitmap = CreateCompatibleBitmap(hdc, MemDC_Rect.right, MemDC_Rect.bottom);
    hMemDC = CreateCompatibleDC(hdc);
    hMemDC_BitmapOld = (HBITMAP)SelectObject(hMemDC, hMemDC_Bitmap);

    PatBlt(hMemDC, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, WHITENESS);

    procFlag = true;
    //trackList = astar.GetTrackList();
    //optimalRoute = astar.GetOptimalRoute();

    for (PathNode<PosType>& node : trackList) {
        //Sleep(100);
        if (!loopProcess) {
            while (!loopProcess) {}
        }
        Cell& cell = grid.cells[node.pos.y][node.pos.x];
        enColor celEncolor = cell.GetEnColor();
        cell.G = to_wstring(node.g);
        cell.H = to_wstring(node.h);
        cell.F = to_wstring(node.f);
        if (celEncolor != RED && celEncolor != BLUE) {
            cell.SetColor(YELLOW);
        }
        cell.SetParentCell(node.parentPos.y, node.parentPos.x);
        Pos<PosType>& parentPos = node.parentPos;
        //if (node.pos.y == parentPos.y) {
        //    if (node.pos.x < parentPos.x) {
        //        for (int x = parentPos.x - 1; x > node.pos.x; x--) {
        //            if(grid.cells[node.pos.y][x].GetEnColor() == WHITE_DEFAULT)
        //                grid.cells[node.pos.y][x].SetColor(BRIGHT_GRAY);
        //        }
        //    }
        //    else {
        //        for (int x = parentPos.x + 1; x < node.pos.x; x++) {
        //            if (grid.cells[node.pos.y][x].GetEnColor() == WHITE_DEFAULT)
        //                grid.cells[node.pos.y][x].SetColor(BRIGHT_GRAY);
        //        }
        //    }
        //}
        //else if (node.pos.x == parentPos.x) {
        //    if (node.pos.y < parentPos.y) {
        //        for (int y = parentPos.y - 1; y > node.pos.y; y--) {
        //            if (grid.cells[y][node.pos.x].GetEnColor() == WHITE_DEFAULT)
        //                grid.cells[y][node.pos.x].SetColor(BRIGHT_GRAY);
        //        }
        //    }
        //    else {
        //        for (int y = parentPos.y + 1; y < node.pos.y; y++) {
        //            if (grid.cells[y][node.pos.x].GetEnColor() == WHITE_DEFAULT)
        //                grid.cells[y][node.pos.x].SetColor(BRIGHT_GRAY);
        //        }
        //    }
        //}
        //else {
        //    if (node.pos.y < parentPos.y) {
        //        int x = parentPos.x;
        //        for (int y = parentPos.y - 1; y > node.pos.y; y--) {
        //            if (node.pos.x < parentPos.x) {
        //                if (grid.cells[y][--x].GetEnColor() == WHITE_DEFAULT)
        //                    grid.cells[y][x].SetColor(BRIGHT_GRAY);
        //            }
        //            else {
        //                if (grid.cells[y][++x].GetEnColor() == WHITE_DEFAULT)
        //                    grid.cells[y][x].SetColor(BRIGHT_GRAY);
        //            }
        //        }
        //    }
        //    else {
        //        int x = parentPos.x;
        //        for (int y = parentPos.y + 1; y < node.pos.y; y++) {
        //            if (node.pos.x < parentPos.x) {
        //                if(grid.cells[y][--x].GetEnColor() == WHITE_DEFAULT)
        //                    grid.cells[y][x].SetColor(BRIGHT_GRAY);
        //            }
        //            else {
        //                if (grid.cells[y][++x].GetEnColor() == WHITE_DEFAULT)
        //                    grid.cells[y][x].SetColor(BRIGHT_GRAY);
        //            }
        //        }
        //    }
        //}

        grid.Draw(hMemDC, MemDC_Rect.right, MemDC_Rect.bottom);
        DrawInformationBox(hMemDC);
        DrawInformationText(hMemDC, colorIndex, startX, startY, destX, destY);
        BitBlt(hdc, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, hMemDC, 0, 0, SRCCOPY);

        while (drawFlag) {};
    }

    if (!pathJps.End()) {

        PathPosition<PosType> pp = *pathJps;
        PosType beforeX = pp.x;
        PosType beforeY = pp.y;
        while (true) {
            if (!loopProcess) {
                while (!loopProcess) {}
            }

            Cell& cell = grid.cells[pp.y][pp.x];
            enColor celEncolor = cell.GetEnColor();

            if (beforeY == cell.y) {
                if (beforeX < cell.x) {
                    for (int x = cell.x - 1; x > beforeX; x--) {
                        grid.cells[beforeY][x].SetColor(BRIGHT_GREEN);
                    }
                }
                else {
                    for (int x = cell.x + 1; x < beforeX; x++) {
                        grid.cells[beforeY][x].SetColor(BRIGHT_GREEN);
                    }
                }
            }
            else if (beforeX == cell.x) {
                if (beforeY < cell.y) {
                    for (int y = cell.y - 1; y > beforeY; y--) {
                        grid.cells[y][beforeX].SetColor(BRIGHT_GREEN);
                    }
                }
                else {
                    for (int y = cell.y + 1; y < beforeY; y++) {
                        grid.cells[y][beforeX].SetColor(BRIGHT_GREEN);
                    }
                }
            }
            else {
                if (beforeY < cell.y) {
                    int x = cell.x;
                    for (int y = cell.y - 1; y > beforeY; y--) {
                        if (beforeX < cell.x) {
                            grid.cells[y][--x].SetColor(BRIGHT_GREEN);
                        }
                        else {
                            grid.cells[y][++x].SetColor(BRIGHT_GREEN);
                        }
                    }
                }
                else {
                    int x = cell.x;
                    for (int y = cell.y + 1; y < beforeY; y++) {
                        if (beforeX < cell.x) {
                            grid.cells[y][--x].SetColor(BRIGHT_GREEN);
                        }
                        else {
                            grid.cells[y][++x].SetColor(BRIGHT_GREEN);
                        }
                    }
                }
            }

            if (celEncolor != RED && celEncolor != BLUE) {
                cell.SetColor(GREEN);

                //grid.Draw(hMemDC, MemDC_Rect.right, MemDC_Rect.bottom);
                //DrawInformationBox(hMemDC);
                //DrawInformationText(hMemDC, colorIndex, startX, startY, destX, destY);
                //BitBlt(hdc, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, hMemDC, 0, 0, SRCCOPY);
                //
                //while (drawFlag) {};
            }

            grid.Draw(hMemDC, MemDC_Rect.right, MemDC_Rect.bottom);
            DrawInformationBox(hMemDC);
            DrawInformationText(hMemDC, colorIndex, startX, startY, destX, destY);
            BitBlt(hdc, 0, 0, MemDC_Rect.right, MemDC_Rect.bottom, hMemDC, 0, 0, SRCCOPY);

            while (drawFlag) {};

            beforeX = cell.x;
            beforeY = cell.y;

            ++pathJps;
            if (pathJps.End()) {
                break;
            }
            else {
                pp = *pathJps;
            }
        }
    }
    else {
        MessageBox(NULL, L"목적지 도달 불가", L"경고", MB_OK | MB_ICONWARNING);
    }

    SelectObject(hMemDC, hMemDC_BitmapOld);
    DeleteObject(hMemDC_Bitmap);
    DeleteObject(hMemDC);

    procFlag = false;
}