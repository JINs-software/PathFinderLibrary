#include "Grid.h"

void Grid::Draw(HDC hdc, int windowWidth, int windowHeight) {
    bool textFlag = false;

    for (size_t i = 0; i < cells.size(); ++i) {
        for (size_t j = 0; j < cells[i].size(); ++j) {
            int x = (cells[i][j].x * cellSize) + offsetX;
            int y = (cells[i][j].y * cellSize) + offsetY;

            // 셀 그리기
            HBRUSH brush = CreateSolidBrush(cells[i][j].GetColorRef());
            RECT rect{ x, y, x + cellSize, y + cellSize };
            FillRect(hdc, &rect, brush);
            DeleteObject(brush);

            // 테두리 그리기
            HPEN pen = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
            HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, GetStockObject(NULL_BRUSH));
            SelectObject(hdc, pen);
            Rectangle(hdc, x, y, x + cellSize, y + cellSize);
    
            // 글자 크기 확인
            SIZE textSize;
            GetTextExtentPoint32(hdc, cells[i][j].XY.c_str(), cells[i][j].XY.length(), &textSize);
            if(textFlag || cellSize > textSize.cx) {
                textFlag = true;
                TextOut(hdc, x + 5, y + 5, cells[i][j].XY.c_str(), cells[i][j].XY.length());
                y += textSize.cy;
                TextOut(hdc, x + 5, y + 5, cells[i][j].GetG().c_str(), cells[i][j].GetG().length());
                y += textSize.cy;
                TextOut(hdc, x + 5, y + 5, cells[i][j].GetH().c_str(), cells[i][j].GetH().length());
                y += textSize.cy;
                TextOut(hdc, x + 5, y + 5, cells[i][j].GetF().c_str(), cells[i][j].GetF().length());
            }

			//// 부모 자식 라인 그리기
			//if (cells[i][j].parentFlag) {
			//	int parentx = cells[i][j].parentX * cellSize + offsetX;
			//	parentx += (cellSize / 2);
			//	int parenty = cells[i][j].parentY * cellSize + offsetY;
			//	parenty += (cellSize / 2);
			//
			//	MoveToEx(hdc, parentx, parenty, NULL);
			//	LineTo(hdc, x + (cellSize / 2), y + (cellSize / 2));
			//}

			SelectObject(hdc, oldBrush);
			DeleteObject(pen);

            // 추가 정보를 사용하여 셀을 그리거나 다양한 작업 수행 가능
        }
    }
}

void Grid::DrawCell(HDC hdc, int windowWidth, int windowHeight, int cY, int cX) {
	bool textFlag = false;

	Cell cell = cells[cY][cX];
	int x = cells[cY][cX].x * cellSize + offsetX;
	int y = cells[cY][cX].y * cellSize + offsetY;

	// 셀 그리기
	HBRUSH brush = CreateSolidBrush(cells[cY][cX].GetColorRef());
	RECT rect{ x, y, x + cellSize, y + cellSize };
	FillRect(hdc, &rect, brush);
	DeleteObject(brush);

	// 테두리 그리기
	HPEN pen = CreatePen(PS_SOLID, 1, RGB(0, 0, 0));
	HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, GetStockObject(NULL_BRUSH));
	SelectObject(hdc, pen);
	Rectangle(hdc, x, y, x + cellSize, y + cellSize);
	SelectObject(hdc, oldBrush);
	DeleteObject(pen);

	// 글자 크기 확인
	SIZE textSize;
	GetTextExtentPoint32(hdc, cells[cY][cX].XY.c_str(), cells[cY][cX].XY.length(), &textSize);
	if (textFlag || cellSize > textSize.cx) {
		textFlag = true;
		TextOut(hdc, x + 5, y + 5, cell.XY.c_str(), cell.XY.length());
		y += textSize.cy;
		TextOut(hdc, x + 5, y + 5, cell.GetG().c_str(), cell.GetG().length());
		y += textSize.cy;
		TextOut(hdc, x + 5, y + 5, cell.GetH().c_str(), cell.GetH().length());
		y += textSize.cy;
		TextOut(hdc, x + 5, y + 5, cell.GetF().c_str(), cell.GetF().length());
	}
}

bool Grid::HandleLeftClick(int mouseY, int mouseX, enColor color) {
	// 반환 값, 색이 변경되었으면 TRUE, 아니면 FALSE
	bool ret = false;
	if ((color == RED && redFlag) || (color == BLUE && blueFlag)) {
		return ret;
	}

	pair<int, int> cellYX = GetCellPos(mouseY, mouseX);
	if (cellYX.first == -1 || cellYX.second == -1) {
		assert(false);
	}
	else {
		enColor celEncolor = cells[cellYX.first][cellYX.second].GetEnColor();
		if (celEncolor == color) {
			if (celEncolor == BLUE) {
				blueFlag = false;
			}
			else if (celEncolor == RED) {
				redFlag = false;
			}
			cells[cellYX.first][cellYX.second].SetColor(WHITE_DEFAULT);
			ret = true;
		}
		else {
			switch (color) {
			case RED:
				if (celEncolor == BLUE) {
					blueFlag = false;
				}
				cells[cellYX.first][cellYX.second].SetColor(RED);
				redFlag = true;
				break;
			case BLUE:
				if (celEncolor == RED) {
					redFlag = false;
				}
				cells[cellYX.first][cellYX.second].SetColor(BLUE);
				blueFlag = true;
				break;
			case BLACK:
				if (celEncolor == BLUE) {
					blueFlag = false;
				}
				else if (celEncolor == RED) {
					redFlag = false;
				}
				cells[cellYX.first][cellYX.second].SetColor(BLACK);
				break;
			case WHITE_DEFAULT:
				if (celEncolor == BLUE) {
					blueFlag = false;
				}
				else if (celEncolor == RED) {
					redFlag = false;
				}
				cells[cellYX.first][cellYX.second].SetColor(WHITE_DEFAULT);
				break;
			}
			ret = true;
		}
	}

	return ret;
}