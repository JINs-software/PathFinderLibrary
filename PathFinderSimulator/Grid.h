#pragma once
#include <vector>
#include <string>
#include <cassert>
#include <Windows.h>

using namespace std;

enum enColor {
	RED,
	BLUE,
	BLACK,
	COLOR_CNT = 3,
	WHITE_DEFAULT,
	YELLOW,
	GREEN,
	BRIGHT_GREEN,
	BRIGHT_GRAY,
	DARK_GRAY
};

class Cell {
public:
	int x;
	int y;
	std::wstring XY = L"";
	std::wstring G = L"";
	std::wstring H = L"";
	std::wstring F = L"";

	bool parentFlag = false;
	int parentX;
	int parentY;

private:
	//std::wstring text;  // 각 셀에 출력할 텍스트
	COLORREF color;     // 셀의 색상
	enColor encolor = WHITE_DEFAULT;

	pair<int, int> parentCellYX;

public:
	Cell(int xPos, int yPos)
		: x(xPos), y(yPos), XY(L"x: " + std::to_wstring(x) + L", y: " + std::to_wstring(y)), color(RGB(255, 255, 255)) {}

	std::wstring GetXY() { return XY; }
	std::wstring GetG() { 
		std::wstring ret = L"g: " + G;
		return ret; 
	}
	std::wstring GetH() {
		std::wstring ret = L"h: " + H;
		return ret;
	}
	std::wstring GetF() {
		std::wstring ret = L"f: " + F;
		return ret;
	}

	void InitGHF() {
		G = L"";
		H = L"";
		F = L"";
	}
	
	void SetColor(enColor encol) {
		encolor = encol;

		switch (encol) {
		case RED:
			color = RGB(255, 0, 0);
			break;
		case BLUE:
			color = RGB(0, 0, 255);
			break;
		case BLACK:
			color = RGB(0, 0, 0);
			break;
		case WHITE_DEFAULT:
			color = RGB(255, 255, 255);
			break;
		case YELLOW:
			color = RGB(0xFF, 0xD4, 0x00);
			break;
		case GREEN:
			color = RGB(0x00, 0x96, 0x30);
			break;
		case BRIGHT_GREEN:
			color = RGB(0x68, 0xff, 0x68);
			break;
		case BRIGHT_GRAY:
			color = RGB(200, 200, 200);
			break;
		case DARK_GRAY:
			color = RGB(44, 44, 44);
			break;
		}
	}
	COLORREF GetColorRef() {
		return color;
	}
	enColor GetEnColor() {
		return encolor;
	}

	void SetParentCell(int Y, int X) {
		parentFlag = true;
		parentY = Y;
		parentX = X;
	}
};

class Grid
{
public:
	int cellSize;
	int offsetX;
	int offsetY;
	int cellX;
	int cellY;
	std::vector<std::vector<Cell>> cells;

	bool redFlag = false;
	bool blueFlag = false;

public:
	Grid() {}
	Grid(int size, int cY, int cX)
		: cellSize(size), cellX(cX), cellY(cY), offsetX(0), offsetY(0)
	{
		for (int i = 0; i < cellY; ++i) {
			std::vector<Cell> row;
			for (int j = 0; j < cellX; ++j) {
				row.emplace_back(j, i);
			}
			cells.emplace_back(row);
		}
	}

	void Clear() {
		redFlag = false;
		blueFlag = false;
		for (int i = 0; i < cells.size(); ++i) {
			for (int j = 0; j < cells[0].size(); ++j) {
				cells[i][j].G = L"";
				cells[i][j].H = L"";
				cells[i][j].F = L"";
				cells[i][j].SetColor(WHITE_DEFAULT);
			}
		}
	}
	void ClearPath() {
		for (int i = 0; i < cells.size(); ++i) {
			for (int j = 0; j < cells[0].size(); ++j) {
				cells[i][j].G = L"";
				cells[i][j].H = L"";
				cells[i][j].F = L"";
				if (cells[i][j].GetEnColor() != BLUE && cells[i][j].GetEnColor() != RED && cells[i][j].GetEnColor() != BLACK) {
					cells[i][j].SetColor(WHITE_DEFAULT);
				}
			}
		}
	}
	void SetGridCell(int size, int cY, int cX) {
		cellSize = size;
		cellX = cX;
		cellY = cY;

		for (int i = 0; i < cellY; ++i) {
			std::vector<Cell> row;
			for (int j = 0; j < cellX; ++j) {
				row.emplace_back(j, i);
			}
			cells.emplace_back(row);
		}
	}

	void Draw(HDC hdc, int windowWidth, int windowHeight);
	void DrawCell(HDC hdc, int windowWidth, int windowHeight, int y, int x);

	void MoveLeft() {
		offsetX += cellSize * 2;
	}

	void MoveRight() {
		offsetX -= cellSize * 2;
	}
	void MoveUp() {
		offsetY += cellSize * 2;
	}

	void MoveDown() {
		offsetY -= cellSize * 2;
	}
	
	void Move(int mouseY, int mouseX) {
		// 현재 선택된 셀이 그대로 그 위치에 존재하길 원한다.
		pair<int, int> YX = GetCellPos(mouseY, mouseX);
		if (YX.first == -1 || YX.second == -1) {
			return;
		}
		Cell cursorCell = cells[YX.first][YX.second];

		offsetY = mouseY - cursorCell.y * cellSize;
		offsetX = mouseX - cursorCell.x * cellSize;
	}

	pair<int, int> GetCellPos(int mouseY, int mouseX) {
		int cellx = (mouseX - offsetX) / cellSize;
		int celly = (mouseY - offsetY) / cellSize;

		if (celly >= 0 && celly < static_cast<int>(cells.size()) &&
			cellx >= 0 && cellx < static_cast<int>(cells[0].size())) {
			return { celly, cellx };
		}
		else {
			return { -1, -1 };
		}
	}

	bool HandleLeftClick(int mouseY, int mouseX, enColor color);
};

