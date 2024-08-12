#pragma once

#include "PathFinder.h"
#include <Windows.h>

//#define DEBUG_MODE
#define DEBUG_MODE2

template<typename T>
class JPSPathFinder : public PathFinder<T> 
{
public:
	using PathFinder<T>::iterator;
	enum DIR {
		UU = 0,
		DD,
		LL,
		RR,
		UL,
		UR,
		DL,
		DR,
		DIR_CNT = 8,
		NONE_DIR	// 방향 없음. JPS 시작점 노드의 부모 노드 방향을 이것으로 설정
	};
	struct JPSNode {
		bool operator<(const JPSNode& other) const {
			return pathNode < other.pathNode;
		}
		bool operator>(const JPSNode& other) const {
			return pathNode > other.pathNode;
		}

		PathNode<T> pathNode;
		DIR dir;
	};
public:
	virtual void Init(T rangeY, T rangeX) override;
	virtual void SetObstacle(T y, T x) override;
	virtual void UnsetObstacle(T y, T x) override;

	virtual typename PathFinder<T>::iterator FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList) override;

private:
	bool checkDestination(const JPSNode& jnode, Pos<T> destPos, DIR dir);
	void createNewNodeLL(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeRR(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeLLRR(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	//void createNewNodeUU(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	//void createNewNodeDD(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeUUDD(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	//void createNewNodeLU(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	//void createNewNodeRD(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeLURD_new(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeLURD(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	//void createNewNodeLD(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	//void createNewNodeRU(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeLDRU_new(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);
	void createNewNodeLDRU(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList);

	// (y, x)는 장애물이 아님을 가정!!,	(y, x) 좌(또는 우) 처음 만나는 장애물 x위치 반환
	T getEndPosOfPath(std::vector<std::vector<BYTE*>>& straightPath, bool leftDir, T y, T x);
	// (y, x)는 장애물임을 가정!!,		(y, x) 좌(또는 우) 처음 만나는 비장애물 x위치 반환
	T getEndPosOfObstacle(std::vector<std::vector<BYTE*>>& straightPath, bool leftDir, T y, T x);

protected:
	using PathFinder<T>::calculate_H;
	using PathFinder<T>::calculate_G;
	using PathFinder<T>::checkAvailability;

protected:
	using PathFinder<T>::m_RangeY;
	using PathFinder<T>::m_RangeX;
	using PathFinder<T>::m_ObstacleMap;
	using PathFinder<T>::m_ChunkLLRR;
	using PathFinder<T>::m_ObstacleBitMapLLRR;

	BYTE* m_ChunkUUDD;	// UU DD
	BYTE* m_ChunkLURD;	// LU RD
	BYTE* m_ChunkLDRU;	// LD RU

	std::vector<std::vector<BYTE*>> m_ObstacleBitMapUUDD;		// UU DD
	std::vector<std::vector<BYTE*>> m_ObstacleBitMapLURD;		// LU RD
	std::vector<std::vector<BYTE*>> m_ObstacleBitMapLDRU;		// LD RU

private:
	Pos<T> coordXYtoUUDD(Pos<T> pos);
	Pos<T> coordXYtoLURD(Pos<T> pos);
	Pos<T> coordXYtoLDRU(Pos<T> pos);
	
	Pos<T> coordUUDDtoXY(Pos<T> pos);
	Pos<T> coordLURDtoXY(Pos<T> pos);
	Pos<T> coordLDRUtoXY(Pos<T> pos);
};

template<typename T>
void JPSPathFinder<T>::Init(T rangeY, T rangeX)
{
	PathFinder<T>::Init(rangeY, rangeX);

	int colLength;
	// UU DD
	colLength = m_RangeY / BYTE_BIT + ((m_RangeY % BYTE_BIT == 0) ? 0 : 1);
	m_ChunkUUDD = new BYTE[m_RangeX * colLength];
	memset(m_ChunkUUDD, 0, m_RangeX * colLength);
	m_ObstacleBitMapUUDD.resize(m_RangeX);
	for (int y = 0; y < m_RangeX; y++) {
		m_ObstacleBitMapUUDD[y].resize(colLength);
		for (int x = 0; x < colLength; x++) {
			m_ObstacleBitMapUUDD[y][x] = m_ChunkUUDD + y * colLength + x;
		}
	}
	if (m_RangeY % BYTE_BIT != 0) {
		int resBit = m_RangeY % BYTE_BIT;
		BYTE mask = 0b1111'1111;
		mask = mask >> resBit;

		for (int y = 0; y < m_RangeX; y++) {
			*m_ObstacleBitMapUUDD[y][colLength - 1] = *m_ObstacleBitMapUUDD[y][colLength - 1] | mask;
		}
	}


	std::vector<BYTE> masks = {
		0b0111'1111,
		0b0011'1111,
		0b0001'1111,
		0b0000'1111,
		0b0000'0111,
		0b0000'0011,
		0b0000'0001,
		0b0000'0000
	};
	int rowLength = m_RangeX + m_RangeY - 1;
	int minRange = min(m_RangeX, m_RangeY);
	colLength = minRange;
	colLength = colLength / BYTE_BIT + ((colLength % BYTE_BIT == 0) ? 0 : 1);

	// LD RU
	m_ChunkLDRU = new BYTE[rowLength * colLength];
	memset(m_ChunkLDRU, 0b1111'1111, rowLength * colLength);
	m_ObstacleBitMapLDRU.resize(rowLength);
	for (int y = 0; y < rowLength; y++) {
		m_ObstacleBitMapLDRU[y].resize(colLength);
		for (int x = 0; x < colLength; x++) {
			m_ObstacleBitMapLDRU[y][x] = m_ChunkLDRU + y * colLength + x;
		}
	}
	for (int y = 0; y <= rowLength / 2; y++) {
		if (y < minRange - 1) {
			BYTE mask = masks[y % BYTE_BIT];
			for (int x = 0; x < (y / BYTE_BIT); x++) {
				*m_ObstacleBitMapLDRU[y][x] = 0;
				*m_ObstacleBitMapLDRU[rowLength - 1 - y][x] = 0;
			}
			*m_ObstacleBitMapLDRU[y][y / BYTE_BIT] = *m_ObstacleBitMapLDRU[y][y / BYTE_BIT] & mask;
			*m_ObstacleBitMapLDRU[rowLength - 1 - y][y / BYTE_BIT] = *m_ObstacleBitMapLDRU[rowLength - 1 - y][y / BYTE_BIT] & mask;
		}
		else {
			BYTE mask = masks[minRange / BYTE_BIT];
			for (int x = 0; x < minRange / BYTE_BIT; x++) {
				*m_ObstacleBitMapLDRU[y][x] = 0;
				*m_ObstacleBitMapLDRU[rowLength - 1 - y][x] = 0;
			}
			*m_ObstacleBitMapLDRU[y][minRange / BYTE_BIT] = *m_ObstacleBitMapLDRU[y][minRange / BYTE_BIT] & mask;
			*m_ObstacleBitMapLDRU[rowLength - 1 - y][minRange / BYTE_BIT] = *m_ObstacleBitMapLDRU[rowLength - 1 - y][minRange / BYTE_BIT] & mask;
		}
	}

	// LU RD
	m_ChunkLURD = new BYTE[rowLength * colLength];
	memset(m_ChunkLURD, 0b1111'1111, rowLength * colLength);
	m_ObstacleBitMapLURD.resize(rowLength);
	for (int y = 0; y < rowLength; y++) {
		m_ObstacleBitMapLURD[y].resize(colLength);
		for (int x = 0; x < colLength; x++) {
			m_ObstacleBitMapLURD[y][x] = m_ChunkLURD + y * colLength + x;
		}
	}
	for (int y = 0; y <= rowLength / 2; y++) {
		if (y < minRange - 1) {
			BYTE mask = masks[y % BYTE_BIT];
			for (int x = 0; x < (y / BYTE_BIT); x++) {
				*m_ObstacleBitMapLURD[y][x] = 0;
				*m_ObstacleBitMapLURD[rowLength - 1 - y][x] = 0;
			}
			*m_ObstacleBitMapLURD[y][y / BYTE_BIT] = *m_ObstacleBitMapLURD[y][y / BYTE_BIT] & mask;
			*m_ObstacleBitMapLURD[rowLength - 1 - y][y / BYTE_BIT] = *m_ObstacleBitMapLURD[rowLength - 1 - y][y / BYTE_BIT] & mask;
		}
		else {
			BYTE mask = masks[minRange / BYTE_BIT];
			for (int x = 0; x < minRange / BYTE_BIT; x++) {
				*m_ObstacleBitMapLURD[y][x] = 0;
				*m_ObstacleBitMapLURD[rowLength - 1 - y][x] = 0;
			}
			*m_ObstacleBitMapLURD[y][minRange / BYTE_BIT] = *m_ObstacleBitMapLURD[y][minRange / BYTE_BIT] & mask;
			*m_ObstacleBitMapLURD[rowLength - 1 - y][minRange / BYTE_BIT] = *m_ObstacleBitMapLURD[rowLength - 1 - y][minRange / BYTE_BIT] & mask;
		}
	}
}

template<typename T>
void JPSPathFinder<T>::SetObstacle(T y, T x)
{
	PathFinder<T>::SetObstacle(y, x);

	// UU DD
	Pos<T> coordUUDD = coordXYtoUUDD({ y, x });
	*m_ObstacleBitMapUUDD[coordUUDD.y][coordUUDD.x / BYTE_BIT] = *m_ObstacleBitMapUUDD[coordUUDD.y][coordUUDD.x / BYTE_BIT] | setMasks[coordUUDD.x % BYTE_BIT];

	// LU RD		
	Pos<T> coordLURD = coordXYtoLURD({ y, x });
	*m_ObstacleBitMapLURD[coordLURD.y][coordLURD.x / BYTE_BIT] = *m_ObstacleBitMapLURD[coordLURD.y][coordLURD.x / BYTE_BIT] | setMasks[coordLURD.x % BYTE_BIT];

	// LD RU
	Pos<T> coordLDRU = coordXYtoLDRU({ y, x });
	*m_ObstacleBitMapLDRU[coordLDRU.y][coordLDRU.x / BYTE_BIT] = *m_ObstacleBitMapLDRU[coordLDRU.y][coordLDRU.x / BYTE_BIT] | setMasks[coordLDRU.x % BYTE_BIT];

#if defined DEBUG_MODE
	cout << "[LLRR]" << endl;
	for (int y = 0; y < m_ObstacleBitMapLLRR.size(); y++) {
		for (int x = 0; x < m_ObstacleBitMapLLRR[y].size(); x++) {
			BYTE* bytePtr = m_ObstacleBitMapLLRR[y][x];
			BYTE byte = *bytePtr;
			BYTE bitMask = 0b1000'0000;
			for (int i = 0; i < 8; i++) {
				if ((byte & bitMask) > 0) {
					cout << "O";
				}
				else {
					cout << "X";
				}

				byte <<= 1;
			}
		}
		cout << endl;
	}
	cout << endl;
	cout << "[UUDD]" << endl;
	for (int y = 0; y < m_ObstacleBitMapUUDD.size(); y++) {
		for (int x = 0; x < m_ObstacleBitMapUUDD[y].size(); x++) {
			BYTE* bytePtr = m_ObstacleBitMapUUDD[y][x];
			BYTE byte = *bytePtr;
			BYTE bitMask = 0b1000'0000;
			for (int i = 0; i < 8; i++) {
				if ((byte & bitMask) > 0) {
					cout << "O";
				}
				else {
					cout << "X";
				}

				byte <<= 1;
			}
		}
		cout << endl;
	}

	cout << endl;
	cout << "[LURD]" << endl;
	for (int y = 0; y < m_ObstacleBitMapLURD.size(); y++) {
		for (int x = 0; x < m_ObstacleBitMapLURD[y].size(); x++) {
			BYTE* bytePtr = m_ObstacleBitMapLURD[y][x];
			BYTE byte = *bytePtr;
			BYTE bitMask = 0b1000'0000;
			for (int i = 0; i < 8; i++) {
				if ((byte & bitMask) > 0) {
					cout << "O";
				}
				else {
					cout << "X";
				}

				byte <<= 1;
			}
		}
		cout << endl;
	}

	cout << endl;
	cout << "[LDRU]" << endl;
	for (int y = 0; y < m_ObstacleBitMapLDRU.size(); y++) {
		for (int x = 0; x < m_ObstacleBitMapLDRU[y].size(); x++) {
			BYTE* bytePtr = m_ObstacleBitMapLDRU[y][x];
			BYTE byte = *bytePtr;
			BYTE bitMask = 0b1000'0000;
			for (int i = 0; i < 8; i++) {
				if ((byte & bitMask) > 0) {
					cout << "O";
				}
				else {
					cout << "X";
				}

				byte <<= 1;
			}
		}
		cout << endl;
	}

	cout << "====================================================================" << endl;
#endif
}

template<typename T>
void JPSPathFinder<T>::UnsetObstacle(T y, T x)
{
	PathFinder<T>::UnsetObstacle(y, x);

	// UU DD
	Pos<T> coordUUDD = coordXYtoUUDD({ y, x });
	*m_ObstacleBitMapUUDD[coordUUDD.y][coordUUDD.x / BYTE_BIT] = *m_ObstacleBitMapUUDD[coordUUDD.y][coordUUDD.x / BYTE_BIT] & unsetMasks[coordUUDD.x % BYTE_BIT];

	// LU RD		
	Pos<T> coordLURD = coordXYtoLURD({ y, x });
	*m_ObstacleBitMapLURD[coordLURD.y][coordLURD.x / BYTE_BIT] = *m_ObstacleBitMapLURD[coordLURD.y][coordLURD.x / BYTE_BIT] & unsetMasks[coordLURD.x % BYTE_BIT];

	// LD RU
	Pos<T> coordLDRU = coordXYtoLDRU({ y, x });
	*m_ObstacleBitMapLDRU[coordLDRU.y][coordLDRU.x / BYTE_BIT] = *m_ObstacleBitMapLDRU[coordLDRU.y][coordLDRU.x / BYTE_BIT] & unsetMasks[coordLDRU.x % BYTE_BIT];
}

template<typename T>
typename PathFinder<T>::iterator JPSPathFinder<T>::FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList)
{
	// 초기화: 탐색을 위한 자료구조 생성
	std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>> openList;
	std::vector<std::vector<bool>> closeList(m_RangeY, std::vector<bool>(m_RangeX, false));
	std::map<Pos<T>, Pos<T>> parentPosMap;
	Pos<T> destPos = { destY, destX };
	bool arriveFlag = false;
	PathNode<T> destNode = { 0 };

	// 초기화: 시작 노드 생성
	PathNode<T> startNode;
	startNode.parentPos = { startY, startX };
	startNode.pos = startNode.parentPos;
	startNode.g = 0;
	startNode.h = calculate_H(startNode.pos, destPos);
	startNode.f = startNode.g + startNode.h;
	openList.push({ startNode, NONE_DIR});

	// 탐색
	while (!openList.empty()) {
		JPSNode jnode = openList.top();
		openList.pop();
		trackList.push_back(jnode.pathNode);
		Pos<T> pos = jnode.pathNode.pos;
		parentPosMap.insert({ pos, jnode.pathNode.parentPos });

		// 직선 또는 대각선 방향으로 도착지가 있는 지 확인
		if (checkDestination(jnode, destPos, jnode.dir)) {
			arriveFlag = true;
			destNode.parentPos = pos;
			destNode.pos = destPos;
			parentPosMap.insert({ destPos, destNode.parentPos });
			std::cout << "도착!" << std::endl;
			break;
		}

		// 탐색 가능한 방향으로 탐색
		// LL  탐색
		if (jnode.dir == NONE_DIR || jnode.dir == LL) {
			if (jnode.pathNode.pos.x > 0 && m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x - 1] == false) {
				createNewNodeLLRR(jnode, true, openList, destPos, parentPosMap, trackList);
			}
		}
		// RR 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == RR) {
			if (jnode.pathNode.pos.x < m_RangeX - 1 && m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x + 1] == false) {
				createNewNodeLLRR(jnode, false, openList, destPos, parentPosMap, trackList);
			}
		}
		// UU 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == UU) {
			if (jnode.pathNode.pos.y > 0 && m_ObstacleMap[jnode.pathNode.pos.y - 1][jnode.pathNode.pos.x] == false) {
				createNewNodeUUDD(jnode, true, openList, destPos, parentPosMap, trackList);
			}
		}
		// DD 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == DD) {
			if (jnode.pathNode.pos.y < m_RangeY - 1 && m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x] == false) {
				createNewNodeUUDD(jnode, false, openList, destPos, parentPosMap, trackList);
			}
		}
		// RD -> LU 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == UL) {
			if (jnode.pathNode.pos.x > 0 && jnode.pathNode.pos.y > 0 && m_ObstacleMap[jnode.pathNode.pos.y - 1][jnode.pathNode.pos.x - 1] == false) {
				createNewNodeLURD(jnode, true, openList, destPos, parentPosMap, trackList);
			}
		}
		// LU -> RD 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == DR) {
			if (jnode.pathNode.pos.x < m_RangeX - 1 && jnode.pathNode.pos.y < m_RangeY - 1 && m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x + 1] == false) {
				createNewNodeLURD(jnode, false, openList, destPos, parentPosMap, trackList);
			}
		}
		// RU -> LD 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == DL) {
			if (jnode.pathNode.pos.x > 0 && jnode.pathNode.pos.y < m_RangeY - 1 && m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x - 1] == false) {
				createNewNodeLDRU(jnode, true, openList, destPos, parentPosMap, trackList);
			}
		}
		// LD -> RU 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == UR) {
			if (jnode.pathNode.pos.x < m_RangeX - 1 && jnode.pathNode.pos.y > 0 && m_ObstacleMap[jnode.pathNode.pos.y - 1][jnode.pathNode.pos.x + 1] == false) {
				createNewNodeLDRU(jnode, false, openList, destPos, parentPosMap, trackList);
			}
		}
	}

	
	//// 탐색 완료
	//if (arriveFlag) {
	//	Pos<T> pos = destNode.pos;
	//	while (true) {
	//		//path << pos;
	//		path.Add(pos);
	//
	//		if (startNode.pos == pos) {
	//			break;
	//		}
	//		else {
	//			pos = parentPosMap[pos];
	//		}
	//	}
	//}

	return iterator();
}

template<typename T>
bool JPSPathFinder<T>::checkDestination(const JPSNode& jnode, Pos<T> destPos, DIR dir)
{
	if (jnode.pathNode.pos.x == destPos.x && jnode.pathNode.pos.y == destPos.y) {
		return true;
	}

	Pos<T> nowPos = jnode.pathNode.pos;
	if ((dir == DIR::LL || dir == DIR::NONE_DIR) && jnode.pathNode.pos.x > destPos.x) {
		T endLL = getEndPosOfPath(m_ObstacleBitMapLLRR, true, nowPos.y, nowPos.x);
		if (destPos.y == nowPos.y && destPos.x > endLL) {
			return true;
		} 
	}
	if ((dir == DIR::RR || dir == DIR::NONE_DIR) && jnode.pathNode.pos.x < destPos.x) {
		T endRR = getEndPosOfPath(m_ObstacleBitMapLLRR, false, nowPos.y, nowPos.x);
		if (destPos.y == nowPos.y && destPos.x < endRR) {
			return true;
		}
	}

	Pos<T> nowUUDD = coordXYtoUUDD(nowPos);
	Pos<T> destUUDD = coordXYtoUUDD(destPos);
	if ((dir == DIR::UU || dir == DIR::NONE_DIR) && nowUUDD.x > destUUDD.x) {
		T endUU = getEndPosOfPath(m_ObstacleBitMapUUDD, true, nowUUDD.y, nowUUDD.x);
		if (destUUDD.y == nowUUDD.y && destUUDD.x > endUU) {
			return true;
		}
	}
	if ((dir == DIR::DD || dir == DIR::NONE_DIR) && nowUUDD.x < destUUDD.x) {
		T endDD = getEndPosOfPath(m_ObstacleBitMapUUDD, false, nowUUDD.y, nowUUDD.x);
		if (destUUDD.y == nowUUDD.y && destUUDD.x < endDD) {
			return true;
		}
	}

	// LU RD
	Pos<T> nowLURD = coordXYtoLURD(nowPos);
	Pos<T> destLURD = coordXYtoLURD(destPos);
	if ((dir == DIR::UL || dir == DIR::NONE_DIR) && nowLURD.x > destLURD.x) {
		T endLU = getEndPosOfPath(m_ObstacleBitMapLURD, true, nowLURD.y, nowLURD.x);
		if (destLURD.y == nowLURD.y && destLURD.x > endLU) {
			return true;
		}
	}
	if ((dir == DIR::DR || dir == DIR::NONE_DIR) && nowLURD.x < destLURD.x) {
		T endRD = getEndPosOfPath(m_ObstacleBitMapLURD, false, nowLURD.y, nowLURD.x);
		if (destLURD.y == nowLURD.y && destLURD.x < endRD) {
			return true;
		}
	}

	// LD RU
	Pos<T> nowLDRU = coordXYtoLDRU(nowPos);
	Pos<T> destLDRU = coordXYtoLDRU(destPos);
	if ((dir == DIR::UR || dir == DIR::NONE_DIR) && nowLDRU.x > destLDRU.x) {
		T endLD = getEndPosOfPath(m_ObstacleBitMapLDRU, true, nowLDRU.y, nowLDRU.x);
		if (destLDRU.y == nowLDRU.y && destLDRU.x > endLD) {
			return true;
		}
	}
	if ((dir == DIR::DL || dir == DIR::NONE_DIR) && nowLDRU.x < destLDRU.x) {
		T endRU = getEndPosOfPath(m_ObstacleBitMapLDRU, false, nowLDRU.y, nowLDRU.x);
		if (destLDRU.y == nowLDRU.y && destLDRU.x < endRU) {
			return true;
		}
	}

	return false;
}

template<typename T>
void JPSPathFinder<T>::createNewNodeLL(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	T x;
	T y = jnode.pathNode.pos.y;

	if (y > 0) {
		x = jnode.pathNode.pos.x;
		while (true) {
			T endObsX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, true, y - 1, x);
			if (endObsX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos.y = y - 1;
			pathNode.pos.x = endObsX;
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ pathNode, UL });
			
			x = getEndPosOfPath(m_ObstacleBitMapLLRR, true, y - 1, endObsX);
			if (x == -1) {
				break;
			}
		}
	}
	
	if (y < m_ObstacleBitMapLLRR.size() - 1) {
		x = jnode.pathNode.pos.x;
		while (true) {
			T endObsX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, true, y + 1, x);
			if (endObsX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos.y = y + 1;
			pathNode.pos.x = endObsX;
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ pathNode, DL });

			x = getEndPosOfPath(m_ObstacleBitMapLLRR, true, y + 1, endObsX);
			if (x == -1) {
				break;
			}
		}
	}
}

template<typename T>
void JPSPathFinder<T>::createNewNodeRR(const JPSNode& jnode, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	T x;
	T y = jnode.pathNode.pos.y;

	if (y > 0) {
		x = jnode.pathNode.pos.x;
		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, false, y - 1, x);
			if (endPosX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos.y = y - 1;
			pathNode.pos.x = endPosX;
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ pathNode, UR });

			x = getEndPosOfPath(m_ObstacleBitMapLLRR, false, y - 1, endPosX);
			if (endPosX == -1) {
				break;
			}
		}
	}

	if (y < m_ObstacleBitMapLLRR.size() - 1) {
		x = jnode.pathNode.pos.x;
		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, false, y + 1, x);
			if (endPosX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos.y = y + 1;
			pathNode.pos.x = endPosX;
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ pathNode, DR });

			x = getEndPosOfPath(m_ObstacleBitMapLLRR, false, y + 1, endPosX);
			if (endPosX == -1) {
				break;
			}
		}
	}
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeLLRR(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	T x;
	T y = jnode.pathNode.pos.y;

	if (y > 0) {
		//x = jnode.pathNode.pos.x;
		x = getEndPosOfPath(m_ObstacleBitMapLLRR, leftDir, y - 1, jnode.pathNode.pos.x);
		if (x == -1) {
			return;
		}

		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, leftDir, y - 1, x);
			if (endPosX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos.y = y - 1;
			pathNode.pos.x = endPosX;
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			if (leftDir) {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UL" << endl;
#endif
				openList.push(JPSNode{ pathNode, UL });
			}
			else {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UR" << endl;
#endif
				openList.push(JPSNode{ pathNode, UR });
			}

			x = getEndPosOfPath(m_ObstacleBitMapLLRR, leftDir, y - 1, endPosX);
			if (x == -1) {
				break;
			}
		}
	}

	if (y < m_ObstacleBitMapLLRR.size() - 1) {
		//x = jnode.pathNode.pos.x;
		x = getEndPosOfPath(m_ObstacleBitMapLLRR, leftDir, y + 1, jnode.pathNode.pos.x);
		if (x == -1) {
			return;
		}

		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, leftDir, y + 1, x);
			if (endPosX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos.y = y + 1;
			pathNode.pos.x = endPosX;
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			if (leftDir) {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DL" << endl;
#endif
				openList.push(JPSNode{ pathNode, DL });
			}
			else {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DR" << endl;
#endif
				openList.push(JPSNode{ pathNode, DR });
			}

			x = getEndPosOfPath(m_ObstacleBitMapLLRR, leftDir, y + 1, endPosX);
			if (x == -1) {
				break;
			}
		}
	}
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeUUDD(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	Pos<T> coordPos = coordXYtoUUDD(jnode.pathNode.pos);
	T x;
	T y = coordPos.y;

	if (y > 0) {
		//x = coordPos.x;
		x = getEndPosOfPath(m_ObstacleBitMapUUDD, leftDir, y - 1, coordPos.x);
		if (x == -1) {
			return;
		}

		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapUUDD, leftDir, y - 1, x);
			if (endPosX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos = coordUUDDtoXY({ y - 1, endPosX });
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			if (leftDir) {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UR" << endl;
#endif
				openList.push(JPSNode{ pathNode, UR });
			}
			else {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DR" << endl;
#endif
				openList.push(JPSNode{ pathNode, DR });
			}

			x = getEndPosOfPath(m_ObstacleBitMapUUDD, leftDir, y - 1, endPosX);
			if (x == -1) {
				break;
			}
		}
	}

	if (y < m_ObstacleBitMapUUDD.size() - 1) {
		//x = coordPos.x;
		x = getEndPosOfPath(m_ObstacleBitMapUUDD, leftDir, y + 1, coordPos.x);
		if (x == -1) {
			return;
		}

		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapUUDD, leftDir, y + 1, x);
			if (endPosX == -1) {
				break;
			}
			PathNode<T> pathNode;
			pathNode.pos = coordUUDDtoXY({ y + 1, endPosX });
			pathNode.g = calculate_G(pathNode, jnode.pathNode);
			pathNode.h = calculate_H(pathNode.pos, destPos);
			pathNode.f = pathNode.g + pathNode.h;
			pathNode.parentPos = jnode.pathNode.pos;
			if (leftDir) {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UL" << endl;
#endif
				openList.push(JPSNode{ pathNode, UL });
			}
			else {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DL" << endl;
#endif
				openList.push(JPSNode{ pathNode, DL });
			}

			x = getEndPosOfPath(m_ObstacleBitMapUUDD, leftDir, y + 1, endPosX);
			if (x == -1) {
				break;
			}
		}
	}
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeLURD_new(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	/*
	Pos<T> coordPos = coordXYtoLURD(jnode.pathNode.pos);
	T x;
	T y = coordPos.y;

	if (leftDir) {
		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y, x);
		// endOfPath - 1 ~ x - 1 까지 확인
		if (y > 0 && x - 1 >= 0) {
			int byteIdx = (x - 1) / BYTE_BIT;
			BYTE bitIdx = (x - 1) % BYTE_BIT;
		}
		if (y < m_ObstacleBitMapLURD.size() - 1) {

		}
	}
	else {

	}
	*/
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeLURD(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	Pos<T> coordPos = coordXYtoLURD(jnode.pathNode.pos);
	T x;
	T y = coordPos.y;
	
	if (leftDir) {		// RD -> LU
		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y, x);
		if (endOfPath == -1) {
			return;
		}
		//if (jnode.pathNode.pos.y == jnode.pathNode.pos.x) {
			if (y > 0 && x - 1 >= 0) {
				//m_ObstacleBitMapLURD[y - 1]
				// 'x - 1' 기점부터 비어있는 비트를 대상으로 UU 새로운 노드 생성
				int byteIdx = (x - 1) / BYTE_BIT;
				BYTE bitIdx = (x - 1) % BYTE_BIT;

				//int endOfByteIdx = endOfPath - 1 >= 0 ? (endOfPath - 1) / BYTE_BIT : 0;

				bool breakFlag = false;
				for (; byteIdx >= 0; byteIdx--) {
					BYTE byte = *m_ObstacleBitMapLURD[y - 1][byteIdx];
					byte = ~byte;
					byte &= rightUnsetMasks[bitIdx];

					BYTE emptyIdx;
					while ((emptyIdx = GetLSBIdx(byte)) != BYTE_BIT) {
						if (byteIdx * BYTE_BIT + emptyIdx <= endOfPath - 1) {
							breakFlag = true;
							break;
						}

						T emptyX = byteIdx * BYTE_BIT + emptyIdx;
						PathNode<T> pathNode;
						pathNode.pos = coordLURDtoXY({ y - 1, emptyX });
						pathNode.g = calculate_G(pathNode, jnode.pathNode);
						pathNode.h = calculate_H(pathNode.pos, destPos);
						pathNode.f = pathNode.g + pathNode.h;
						pathNode.parentPos = jnode.pathNode.pos;
						openList.push(JPSNode{ pathNode, UU });
#if defined DEBUG_MODE2
						cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UU" << endl;
#endif


						byte = byte & unsetMasks[emptyIdx];
					}
					if (breakFlag) {
						break;
					}

					bitIdx = 0;
				}
			}
			if (y < m_ObstacleBitMapLURD.size() - 1 && x - 1 >= 0) {
				//m_ObstacleBitMapLURD[y + 1]
				// 'x - 1' 기점부터 비어있는 비트를 대상으로 LL 새로운 노드 생성
				int byteIdx = (x - 1) / BYTE_BIT;
				BYTE bitIdx = (x - 1) % BYTE_BIT;
				int endOfByteIdx = endOfPath - 1 >= 0 ? (endOfPath - 1) / BYTE_BIT : 0;

				bool breakFlag = false;
				for (; byteIdx >= endOfByteIdx; byteIdx--) {
					BYTE byte = *m_ObstacleBitMapLURD[y + 1][byteIdx];
					byte = ~byte;
					byte &= rightUnsetMasks[bitIdx];

					BYTE emptyIdx;
					while ((emptyIdx = GetLSBIdx(byte)) != BYTE_BIT) {
						if (byteIdx * BYTE_BIT + emptyIdx <= endOfPath - 1) {
							breakFlag = true;
							break;
						}
						T emptyX = byteIdx * BYTE_BIT + emptyIdx;
						PathNode<T> pathNode;
						pathNode.pos = coordLURDtoXY({ y + 1, emptyX });
						pathNode.g = calculate_G(pathNode, jnode.pathNode);
						pathNode.h = calculate_H(pathNode.pos, destPos);
						pathNode.f = pathNode.g + pathNode.h;
						pathNode.parentPos = jnode.pathNode.pos;
						openList.push(JPSNode{ pathNode, LL });
#if defined DEBUG_MODE2
						cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: LL" << endl;
#endif

						byte = byte & unsetMasks[emptyIdx];
					}
					if (breakFlag) {
						break;
					}

					bitIdx = 0;
				}
			}
		//}
		//else if (jnode.pathNode.pos.y < jnode.pathNode.pos.x) {
		//	if (y > 0) {
		//		//m_ObstacleBitMapLURD[y - 1]
		//		// 'x - 1' 기점부터 비어있는 비트를 대상으로 UU 새로운 노드 생성
		//	}
		//	if (y < m_ObstacleBitMapLURD.size() - 1) {
		//		//m_ObstacleBitMapLURD[y + 1]
		//		// 'x' 기점부터 비어있는 비트를 대상으로 LL 새로운 노드 생성
		//	}
		//}
		//else {
		//	if (y > 0) {
		//		//m_ObstacleBitMapLURD[y - 1]
		//		// 'x' 기점부터 비어있는 비트를 대상으로 UU 새로운 노드 생성
		//	}
		//	if (y < m_ObstacleBitMapLURD.size() - 1) {
		//		//m_ObstacleBitMapLURD[y + 1]
		//		// 'x - 1' 기점부터 비어있는 비트를 대상으로 LL 새로운 노드 생성
		//	}
		//}
	}
	else {				// LU -> RD
		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y, x);
		if (endOfPath == -1) {
			return;
		}
		//if (jnode.pathNode.pos.y == jnode.pathNode.pos.x) {
			if (y > 0) {
				//m_ObstacleBitMapLURD[y - 1]
				// 'x' 기점부터 endOfPath까지 비어있는 비트를 대상으로 RR 새로운 노드 생성
				int byteIdx = x / BYTE_BIT;
				BYTE bitIdx = x % BYTE_BIT;
				int endOfByteIdx = endOfPath / BYTE_BIT < m_ObstacleBitMapLURD[y - 1].size() - 1 ? endOfPath / BYTE_BIT : m_ObstacleBitMapLURD[y - 1].size() - 1;
				
				bool breakFlag = false;
				for (; byteIdx <= endOfByteIdx; byteIdx++) {
					BYTE byte = *m_ObstacleBitMapLURD[y - 1][byteIdx];
					byte = ~byte;
					byte &= leftUnsetMasks[bitIdx];

					BYTE emptyIdx;
					while ((emptyIdx = GetMSBIdx(byte)) != BYTE_BIT) {
						if (endOfPath <= byteIdx * BYTE_BIT + emptyIdx) {
							breakFlag = true;
							break;
						}

						T emptyX = byteIdx * BYTE_BIT + emptyIdx;
						PathNode<T> pathNode;
						pathNode.pos = coordLURDtoXY({ y - 1, emptyX });
						pathNode.g = calculate_G(pathNode, jnode.pathNode);
						pathNode.h = calculate_H(pathNode.pos, destPos);
						pathNode.f = pathNode.g + pathNode.h;
						pathNode.parentPos = jnode.pathNode.pos;
						openList.push(JPSNode{ pathNode, RR });
#if defined DEBUG_MODE2
						cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: RR" << endl;
#endif

						byte = byte & unsetMasks[emptyIdx];
					}
					if (breakFlag) {
						break;
					}

					bitIdx = 0;
				}
			}
			if (y < m_ObstacleBitMapLURD.size() - 1) {
				//m_ObstacleBitMapLURD[y + 1]
				// 'x' 기점부터 endOfPath까지 비어있는 비트를 대상으로 DD 새로운 노드 생성
				int byteIdx = x / BYTE_BIT;
				BYTE bitIdx = x % BYTE_BIT;
				int endOfByteIdx = endOfPath / BYTE_BIT < m_ObstacleBitMapLURD[y + 1].size() - 1 ? endOfPath / BYTE_BIT : m_ObstacleBitMapLURD[y + 1].size() - 1;

				bool breakFlag = false;
				for (; byteIdx <= endOfByteIdx; byteIdx++) {
					BYTE byte = *m_ObstacleBitMapLURD[y + 1][byteIdx];
					byte = ~byte;
					byte &= leftUnsetMasks[bitIdx];

					BYTE emptyIdx;
					while ((emptyIdx = GetMSBIdx(byte)) != BYTE_BIT) {
						if (endOfPath <= byteIdx * BYTE_BIT + emptyIdx) {
							breakFlag = true;
							break;
						}

						T emptyX = byteIdx * BYTE_BIT + emptyIdx;
						PathNode<T> pathNode;
						pathNode.pos = coordLURDtoXY({ y + 1, emptyX });
						pathNode.g = calculate_G(pathNode, jnode.pathNode);
						pathNode.h = calculate_H(pathNode.pos, destPos);
						pathNode.f = pathNode.g + pathNode.h;
						pathNode.parentPos = jnode.pathNode.pos;
						openList.push(JPSNode{ pathNode, DD });
#if defined DEBUG_MODE2
						cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DD" << endl;
#endif

						byte = byte & unsetMasks[emptyIdx];
					}
					if (breakFlag) {
						break;
					}

					bitIdx = 0;
				}
			}
		//}
		//else if (jnode.pathNode.pos.y < jnode.pathNode.pos.x) {
		//	if (y > 0) {
		//		//m_ObstacleBitMapLURD[y - 1]
		//		// 'x' 기점부터 비어있는 비트를 대상으로 RR 새로운 노드 생성
		//	}
		//	if (y < m_ObstacleBitMapLURD.size() - 1) {
		//		//m_ObstacleBitMapLURD[y + 1]
		//		// 'x + 1' 기점부터 비어있는 비트를 대상으로 DD 새로운 노드 생성
		//	}
		//}
		//else {
		//	if (y > 0) {
		//		//m_ObstacleBitMapLURD[y - 1]
		//		// 'x + 1' 기점부터 비어있는 비트를 대상으로 RR 새로운 노드 생성
		//	}
		//	if (y < m_ObstacleBitMapLURD.size() - 1) {
		//		//m_ObstacleBitMapLURD[y + 1]
		//		// 'x' 기점부터 비어있는 비트를 대상으로 DD 새로운 노드 생성
		//	}
		//}
	}
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeLDRU(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	Pos<T> coordPos = coordXYtoLDRU(jnode.pathNode.pos);
	T x;
	T y = coordPos.y;

	if (leftDir) {	// RU -> LD
		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y, x);
		if (endOfPath == -1) {
			return;
		}

		if (y > 0 && x - 1 >= 0) {
			int byteIdx = (x - 1) / BYTE_BIT;
			BYTE bitIdx = (x - 1) % BYTE_BIT;

			int endOfByteIdx = endOfPath - 1 >= 0 ? (endOfPath - 1) / BYTE_BIT : 0;

			bool breakFlag = false;
			for (; byteIdx >= endOfByteIdx; byteIdx--) {
				BYTE byte = *m_ObstacleBitMapLDRU[y - 1][byteIdx];
				byte = ~byte;
				byte &= rightUnsetMasks[bitIdx];

				BYTE emptyIdx;
				while ((emptyIdx = GetLSBIdx(byte)) != BYTE_BIT) {
					if (byteIdx * BYTE_BIT + emptyIdx <= endOfPath - 1) {
						breakFlag = true;
						break;
					}

					T emptyX = byteIdx * BYTE_BIT + emptyIdx;
					PathNode<T> pathNode;
					pathNode.pos = coordLDRUtoXY({ y - 1, emptyX });
					pathNode.g = calculate_G(pathNode, jnode.pathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = jnode.pathNode.pos;
					openList.push(JPSNode{ pathNode, LL });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: LL" << endl;
#endif

					byte = byte & unsetMasks[emptyIdx];
				}
				if (breakFlag) {
					break;
				}

				bitIdx = 0;
			}
			if (y < m_ObstacleBitMapLDRU.size() - 1 && x - 1 >= 0) {
				int byteIdx = (x - 1) / BYTE_BIT;
				BYTE bitIdx = (x - 1) % BYTE_BIT;
				int endOfByteIdx = endOfPath - 1 >= 0 ? (endOfPath - 1) / BYTE_BIT : 0;

				bool breakFlag = false;
				for (; byteIdx >= endOfByteIdx; byteIdx--) {
					BYTE byte = *m_ObstacleBitMapLDRU[y + 1][byteIdx];
					byte = ~byte;
					byte &= rightUnsetMasks[bitIdx];

					BYTE emptyIdx;
					while ((emptyIdx = GetLSBIdx(byte)) != BYTE_BIT) {
						if (byteIdx * BYTE_BIT + emptyIdx <= endOfPath - 1) {
							breakFlag = true;
							break;
						}
						T emptyX = byteIdx * BYTE_BIT + emptyIdx;
						PathNode<T> pathNode;
						pathNode.pos = coordLDRUtoXY({ y + 1, emptyX });
						pathNode.g = calculate_G(pathNode, jnode.pathNode);
						pathNode.h = calculate_H(pathNode.pos, destPos);
						pathNode.f = pathNode.g + pathNode.h;
						pathNode.parentPos = jnode.pathNode.pos;
						openList.push(JPSNode{ pathNode, DD });
#if defined DEBUG_MODE2
						cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DD" << endl;
#endif

						byte = byte & unsetMasks[emptyIdx];
					}
					if (breakFlag) {
						break;
					}

					bitIdx = 0;
				}
			}
		}
	}
	else {
		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y, x);
		if (endOfPath == -1) {
			return;
		}

		//if (jnode.pathNode.pos.y == jnode.pathNode.pos.x) {
		if (y > 0) {
			//m_ObstacleBitMapLURD[y - 1]
			// 'x' 기점부터 endOfPath까지 비어있는 비트를 대상으로 RR 새로운 노드 생성
			int byteIdx = x / BYTE_BIT;
			BYTE bitIdx = x % BYTE_BIT;
			int endOfByteIdx = endOfPath / BYTE_BIT < m_ObstacleBitMapLDRU[y - 1].size() - 1 ? endOfPath / BYTE_BIT : m_ObstacleBitMapLDRU[y - 1].size() - 1;

			bool breakFlag = false;
			for (; byteIdx <= endOfByteIdx; byteIdx++) {
				BYTE byte = *m_ObstacleBitMapLDRU[y - 1][byteIdx];
				byte = ~byte;
				byte &= leftUnsetMasks[bitIdx];

				BYTE emptyIdx;
				while ((emptyIdx = GetMSBIdx(byte)) != BYTE_BIT) {
					if (endOfPath <= byteIdx * BYTE_BIT + emptyIdx) {
						breakFlag = true;
						break;
					}

					T emptyX = byteIdx * BYTE_BIT + emptyIdx;
					PathNode<T> pathNode;
					pathNode.pos = coordLDRUtoXY({ y - 1, emptyX });
					pathNode.g = calculate_G(pathNode, jnode.pathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = jnode.pathNode.pos;
					openList.push(JPSNode{ pathNode, UU });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UU" << endl;
#endif

					byte = byte & unsetMasks[emptyIdx];
				}
				if (breakFlag) {
					break;
				}

				bitIdx = 0;
			}
		}
		if (y < m_ObstacleBitMapLDRU.size() - 1) {
			//m_ObstacleBitMapLURD[y + 1]
			// 'x' 기점부터 endOfPath까지 비어있는 비트를 대상으로 DD 새로운 노드 생성
			int byteIdx = x / BYTE_BIT;
			BYTE bitIdx = x % BYTE_BIT;
			int endOfByteIdx = endOfPath / BYTE_BIT < m_ObstacleBitMapLDRU[y + 1].size() - 1 ? endOfPath / BYTE_BIT : m_ObstacleBitMapLDRU[y + 1].size() - 1;

			bool breakFlag = false;
			for (; byteIdx <= endOfByteIdx; byteIdx++) {
				BYTE byte = *m_ObstacleBitMapLDRU[y + 1][byteIdx];
				byte = ~byte;
				byte &= leftUnsetMasks[bitIdx];

				BYTE emptyIdx;
				while ((emptyIdx = GetMSBIdx(byte)) != BYTE_BIT) {
					if (endOfPath <= byteIdx * BYTE_BIT + emptyIdx) {
						breakFlag = true;
						break;
					}

					T emptyX = byteIdx * BYTE_BIT + emptyIdx;
					PathNode<T> pathNode;
					pathNode.pos = coordLDRUtoXY({ y + 1, emptyX });
					pathNode.g = calculate_G(pathNode, jnode.pathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = jnode.pathNode.pos;
					openList.push(JPSNode{ pathNode, RR });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: RR" << endl;
#endif

					byte = byte & unsetMasks[emptyIdx];
				}
				if (breakFlag) {
					break;
				}

				bitIdx = 0;
			}
		}
	}
}

template<typename T>
T JPSPathFinder<T>::getEndPosOfPath(std::vector<std::vector<BYTE*>>& straightPath, bool leftDir, T y, T x)
{
	int byteIdx = x / BYTE_BIT;
	BYTE bitIdx = x % BYTE_BIT;
	BYTE byte = *straightPath[y][byteIdx];

	T ret = -1;

	if (leftDir) {	// 좌측 직선 방향
		// x 위치의 비트 인덱스가 포함된 바이트와 해당 바이트의 MSB 체크
		if (byte != 0 && GetMSBIdx(byte) < bitIdx) {	// byte == 0 이라면 장애물이 없다는 뜻
														// GetMSBIdx(byte) < bitIdx 라면 시작 바이트에 장애물이 있다는 뜻
			byte = byte >> (BYTE_BIT - bitIdx);			
			byte = byte << (BYTE_BIT - bitIdx);

			ret = (T)byteIdx * BYTE_BIT;
			ret += (T)GetLSBIdx(byte);
		}
		else {
			for (byteIdx -= 1; byteIdx >= 0; byteIdx--) {
				byte = *straightPath[y][byteIdx];
				if (byte != 0) {
					ret = (T)byteIdx * BYTE_BIT;
					ret += (T)GetLSBIdx(byte);
					break;
				}
			}
		}
	}
	else {			// 우측 직선 방향
		// x 위치의 비트 인덱스가 포함된 바이트와 해당 바이트의 LSB 체크
		if (byte != 0 && bitIdx < GetLSBIdx(byte)) {
			byte = byte << (bitIdx + 1);
			byte = byte >> (bitIdx + 1);

			ret = (T)byteIdx * BYTE_BIT;
			ret += (T)GetMSBIdx(byte);
		}
		else {
			for (byteIdx += 1; byteIdx < straightPath[0].size(); byteIdx++) {
				byte = *straightPath[y][byteIdx];
				if (byte != 0) {
					ret = (T)byteIdx * BYTE_BIT;
					ret += (T)GetMSBIdx(byte);
					break;
				}
			}
		}
	}

	return ret;
}

template<typename T>
T JPSPathFinder<T>::getEndPosOfObstacle(std::vector<std::vector<BYTE*>>& straightPath, bool leftDir, T y, T x)
{
	int byteIdx = x / BYTE_BIT;
	BYTE bitIdx = x % BYTE_BIT;
	BYTE byte = *straightPath[y][byteIdx];
	byte = ~byte;

	T ret = -1;

	// 1101'1101
	// ~>
	// 0010'0010

	if (leftDir) {	// 좌측 직선 방향
		// x위치의 비트 인덱스가 포함된 바이트와 해당 바이트의 반전 바이트의 MSB 체크
		if (byte != 0 && GetMSBIdx(byte) < bitIdx) {
			byte = byte >> (BYTE_BIT - bitIdx);
			byte = byte << (BYTE_BIT - bitIdx);

			ret = (T)byteIdx * BYTE_BIT;
			ret += (T)GetLSBIdx(byte);
		}
		else {
			for (byteIdx -= 1; byteIdx >= 0; byteIdx--) {
				byte = *straightPath[y][byteIdx];
				byte = ~byte;
				if (byte != 0) {
					ret = (T)byteIdx * BYTE_BIT;
					ret += (T)GetLSBIdx(byte);
					break;
				}
			}
		}
	}
	else {			// 우측 직선 방향
		// x위치의 비트 인덱스가 포함된 바이트와 해당 바이트의 반전 바이트의 LSB 체크
		if (byte != 0 && bitIdx < GetLSBIdx(byte)) {
			byte = byte << (bitIdx + 1);
			byte = byte >> (bitIdx + 1);

			ret = (T)byteIdx * BYTE_BIT;
			ret += (T)GetMSBIdx(byte);
		}
		else {
			for (byteIdx += 1; byteIdx < straightPath[0].size(); byteIdx++) {
				byte = *straightPath[y][byteIdx];
				byte = ~byte;
				if (byte != 0) {
					ret = (T)byteIdx * BYTE_BIT;
					ret += (T)GetMSBIdx(byte);
					break;
				}
			}
		}
	}

	return ret;
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordXYtoUUDD(Pos<T> pos)
{
	return Pos<T>{m_RangeX - 1 - pos.x, pos.y};
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordXYtoLURD(Pos<T> pos)
{
	Pos<T> coordPos;
	coordPos.y = m_RangeX - 1 - pos.x + pos.y;
	if (pos.x >= pos.y) {
		coordPos.x = pos.y;
	}
	else {
		coordPos.x = pos.x;
	}

	return coordPos;
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordXYtoLDRU(Pos<T> pos)
{
	T ry = pos.x;
	T rx = m_RangeY - 1 - pos.y;
	//return coordXYtoLURD(Pos<T>{ry, rx});

	Pos<T> coordPos;
	coordPos.y = m_RangeY - 1 - rx + ry;
	if (rx >= ry) {
		coordPos.x = ry;
	}
	else {
		coordPos.x = rx;
	}

	return coordPos;
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordUUDDtoXY(Pos<T> pos)
{
	// nY = m_RangeX - 1 - x
	// nX = y;
	return Pos<T>{pos.x, m_RangeX - 1 - pos.y};
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordLURDtoXY(Pos<T> pos)
{
	// nY = m_RangeX - 1 - x + y
	// <-> ny - m_RangeX + 1 = y - x
	// <-> x - y = m_RangeX - 1 - nY
	Pos<T> coordPos;
	if (m_RangeX - 1 - pos.y >= 0) {		// x >= y
		// nX = y;
		coordPos.y = pos.x;
		coordPos.x = coordPos.y + m_RangeX - 1 - pos.y;
	}
	else {							
		// nX = x;
		coordPos.x = pos.x;
		coordPos.y = coordPos.x - m_RangeX + 1 + pos.y;
	}
	return coordPos;
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordLDRUtoXY(Pos<T> pos)
{
	//T ry = pos.x;
	//T rx = m_RangeY - 1 - pos.y;
	////return coordXYtoLURD(Pos<T>{ry, rx});
	//
	//Pos<T> coordPos;
	//coordPos.y = m_RangeY - 1 - rx + ry;
	//if (rx >= ry) {
	//	coordPos.x = ry;
	//}
	//else {
	//	coordPos.x = rx;
	//}

	
	Pos<T> coordPos;
	T ry, rx;

	if (m_RangeY - 1 - pos.y >= 0) {
		ry = pos.x;
		rx = m_RangeY - 1 - pos.y + ry;
	}
	else {
		rx = pos.x;
		ry = rx - m_RangeY + 1 + pos.y;
	}

	coordPos.x = ry;
	coordPos.y = m_RangeY - 1 - rx;

	return coordPos;
}
