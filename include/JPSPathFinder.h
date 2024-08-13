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

	std::vector<std::vector<bool>> m_ObstacleMapUUDD;

	std::vector<std::vector<BYTE*>> m_ObstacleBitMapUUDD;		// UU DD
	std::vector<std::vector<BYTE*>> m_ObstacleBitMapLURD;		// LU RD
	std::vector<std::vector<BYTE*>> m_ObstacleBitMapLDRU;		// LD RU

private:
	Pos<T> coordXYtoUUDD(Pos<T> pos);
	Pos<T> coordXYtoLURD(Pos<T> pos, bool axisTransform = false);
	Pos<T> coordXYtoLDRU(Pos<T> pos, bool axisTransform = false);
	
	Pos<T> coordUUDDtoXY(Pos<T> pos);
	Pos<T> coordLURDtoXY(Pos<T> pos, bool axisTransform = false);
	Pos<T> coordLDRUtoXY(Pos<T> pos, bool axisTransform = false);
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
	m_ObstacleMapUUDD.resize(m_RangeX);
	m_ObstacleBitMapUUDD.resize(m_RangeX);
	for (int y = 0; y < m_RangeX; y++) {
		m_ObstacleMapUUDD[y].resize(m_RangeY, false);
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
	m_ObstacleMapUUDD[coordUUDD.y][coordUUDD.x] = true;
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
	m_ObstacleMapUUDD[coordUUDD.y][coordUUDD.x] = false;
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
#if defined DEBUG_MODE2
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
				createNewNodeUUDD(jnode, false, openList, destPos, parentPosMap, trackList);
			}
		}
		// DD 탐색
		if (jnode.dir == NONE_DIR || jnode.dir == DD) {
			if (jnode.pathNode.pos.y < m_RangeY - 1 && m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x] == false) {
				createNewNodeUUDD(jnode, true, openList, destPos, parentPosMap, trackList);
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
	if ((dir == DIR::LL || dir == DIR::NONE_DIR) && (!m_ObstacleMap[y][x - 1]) && destPos.y == nowPos.y && destPos.x < nowPos.x) {
		T endLL = getEndPosOfPath(m_ObstacleBitMapLLRR, true, nowPos.y, nowPos.x);
		if (endLL < destPos.x) {
			return true;
		} 
	}
	if ((dir == DIR::RR || dir == DIR::NONE_DIR) && (!m_ObstacleMap[y][x + 1]) && destPos.y == nowPos.y && jnode.pathNode.pos.x < destPos.x) {
		T endRR = getEndPosOfPath(m_ObstacleBitMapLLRR, false, nowPos.y, nowPos.x);
		if (destPos.x < endRR) {
			return true;
		}
	}

	Pos<T> nowUUDD = coordXYtoUUDD(nowPos);
	Pos<T> destUUDD = coordXYtoUUDD(destPos);
	if ((dir == DIR::UU || dir == DIR::NONE_DIR) && (!m_ObstacleMap[y - 1][x]) && destUUDD.y == nowUUDD.y && nowUUDD.x < destUUDD.x) {
		T endUU = getEndPosOfPath(m_ObstacleBitMapUUDD, false, nowUUDD.y, nowUUDD.x);
		if (destUUDD.x < endUU) {
			return true;
		}
	}
	if ((dir == DIR::DD || dir == DIR::NONE_DIR) && (!m_ObstacleMap[y + 1][x]) && destUUDD.y == nowUUDD.y && destUUDD.x < nowUUDD.x) {
		T endDD = getEndPosOfPath(m_ObstacleBitMapUUDD, true, nowUUDD.y, nowUUDD.x);
		if (endDD < destUUDD.x) {
			return true;
		}
	}

	// LU RD
	Pos<T> nowLURD = coordXYtoLURD(nowPos);
	Pos<T> destLURD = coordXYtoLURD(destPos);
	if ((dir == DIR::UL || dir == DIR::NONE_DIR) && (!m_ObstacleMap[nowPos.y][nowPos.x-1] && !m_ObstacleMap[nowPos.y - 1][nowPos.x]) && destPos.y <= nowPos.y && destPos.x <= nowPos.x) {	// 조건 수정 중......@@@@@@@@@@@@
		T endLU = getEndPosOfPath(m_ObstacleBitMapLURD, true, nowLURD.y, nowLURD.x);
		if (destLURD.y == nowLURD.y) {
			if (destLURD.x > endLU) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			Pos<T> endPos = coordLURDtoXY({ nowLURD.y, endLU });
			Pos<T> destUUDD = coordXYtoUUDD(destPos);
			//for (int y = nowPos.y - 1; y > endPos.y; y--) {
			//	for (int x = nowPos.x - 1; x > endPos.x; x--) {
			for (int y = nowPos.y - 1, x = nowPos.x - 1; y > endPos.y || x > endPos.x; y--, x--) {
				// 좌 탐색
				T endLL = getEndPosOfPath(m_ObstacleBitMapLLRR, true, y, x);
				if (destPos.y == nowPos.y && destPos.x > endLL) {
					return true;
				}

				// 상 탐색
				Pos<T> uudd = coordXYtoUUDD({ y, x });
				T endUU = getEndPosOfPath(m_ObstacleBitMapUUDD, false, uudd.y, uudd.x);
				if (destUUDD.y == uudd.y && destUUDD.x < endUU) {
					return true;
				}
			}
			//	}
			//}
		}
	}
	if ((dir == DIR::DR || dir == DIR::NONE_DIR) && (!m_ObstacleMap[nowPos.y][nowPos.x + 1] && !m_ObstacleMap[nowPos.y + 1][nowPos.x]) && nowPos.x <= destPos.x && nowPos.y <= destPos.y) {
		T endRD = getEndPosOfPath(m_ObstacleBitMapLURD, false, nowLURD.y, nowLURD.x);
		if (destLURD.y == nowLURD.y) {
			if (destLURD.x < endRD) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			Pos<T> endPos = coordLURDtoXY({ nowLURD.y, endRD });
			Pos<T> destUUDD = coordXYtoUUDD(destPos);
			//for (int y = nowPos.y + 1; y < endPos.y; y++) {
			//	for (int x = nowPos.x + 1; x < endPos.x; x++) {
			for (int y = nowPos.y + 1, x = nowPos.x + 1; y < endPos.y || x < endPos.x; y++, x++) {
				// 우 탐색
				T endRR = getEndPosOfPath(m_ObstacleBitMapLLRR, false, y, x);
				if (destPos.y == nowPos.y && destPos.x < endRR) {
					return true;
				}

				// 하 탐색
				Pos<T> uudd = coordXYtoUUDD({ y, x });
				T endDD = getEndPosOfPath(m_ObstacleBitMapUUDD, true, uudd.y, uudd.x);
				if (destUUDD.y == nowUUDD.y && destUUDD.x > endDD) {
					return true;
				}
			}
			//	}
			//}
		}
	}

	// LD RU
	Pos<T> nowLDRU = coordXYtoLDRU(nowPos);
	Pos<T> destLDRU = coordXYtoLDRU(destPos);
	if ((dir == DIR::UR || dir == DIR::NONE_DIR) && (!m_ObstacleMap[nowPos.y][nowPos.x + 1] && !m_ObstacleMap[nowPos.y - 1][nowPos.x]) && destPos.y <= nowPos.y && nowPos.x <= destPos.x) {
		T endRU = getEndPosOfPath(m_ObstacleBitMapLDRU, false, nowLDRU.y, nowLDRU.x);
		if (destLDRU.y == nowLDRU.y) {
			if (destLDRU.x < endRU) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			Pos<T> endPos = coordLDRUtoXY({ nowLDRU.y, endRU });
			Pos<T> destUUDD = coordXYtoUUDD(destPos);
			//for (int y = nowPos.y - 1; y > endPos.y; y--) {
			//	for (int x = nowPos.x + 1; x < endPos.x; x++) {
			for (int y = nowPos.y - 1, x = nowPos.x + 1; y > endPos.y || x < endPos.x; y--, x++) {
				// 우 탐색
				T endRR = getEndPosOfPath(m_ObstacleBitMapLLRR, false, y, x);
				if (destPos.y == nowPos.y && destPos.x < endRR) {
					return true;
				}

				// 상 탐색
				Pos<T> uudd = coordXYtoUUDD({ y, x });
				T endUU = getEndPosOfPath(m_ObstacleBitMapUUDD, false, uudd.y, uudd.x);
				if (destUUDD.y == uudd.y && destUUDD.x < endUU) {
					return true;
				}
			}
			//	}
			//}
		}
	}
	if ((dir == DIR::DL || dir == DIR::NONE_DIR) && (!m_ObstacleMap[nowPos.y][nowPos.x - 1] && !m_ObstacleMap[nowPos.y + 1][nowPos.x]) && nowPos.y <= destPos.y && destPos.x <= nowPos.x) {
		T endLD = getEndPosOfPath(m_ObstacleBitMapLDRU, true, nowLDRU.y, nowLDRU.x);
		Pos<T> destUUDD = coordXYtoUUDD(destPos);
		if (destLDRU.y == nowLDRU.y) {
			if (destLDRU.x > endLD) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			Pos<T> endPos = coordLDRUtoXY({ nowLDRU.y, endLD });
			//for (int y = nowPos.y + 1; y < endPos.y; y++) {
			//	for (int x = nowPos.x - 1; x > endPos.x; x--) {
			for (int y = nowPos.y + 1, x = nowPos.x - 1; y < endPos.y || x > endPos.x; y++, x--) {
				// 하 탐색
				Pos<T> uudd = coordXYtoUUDD({ y, x });
				T endDD = getEndPosOfPath(m_ObstacleBitMapUUDD, true, uudd.y, uudd.x);
				if (destUUDD.y == nowUUDD.y && destUUDD.x > endDD) {
					return true;
				}

				// 좌 탐색
				T endLL = getEndPosOfPath(m_ObstacleBitMapLLRR, true, y, x);
				if (destPos.y == nowPos.y && destPos.x > endLL) {
					return true;
				}
			}
			//	}
			//}
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
		
		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapLLRR, leftDir, y - 1, x);
			if (endPosX == -1 || endPosX == 8) {
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
			if (endPosX == -1 || endPosX == 8) {
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
		
		while (true) {
			T endPosX = getEndPosOfObstacle(m_ObstacleBitMapUUDD, leftDir, y - 1, x);
			if (endPosX == -1 || endPosX == 8) {
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
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DL" << endl;
#endif
				openList.push(JPSNode{ pathNode, DL });
			}
			else {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UL" << endl;
#endif
				openList.push(JPSNode{ pathNode, UL });
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
			if (endPosX == -1 || endPosX == 8) {
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
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DR" << endl;
#endif
				openList.push(JPSNode{ pathNode, DR });
			}
			else {
#if defined DEBUG_MODE2
				cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UR" << endl;
#endif
				openList.push(JPSNode{ pathNode, UR });
			}

			x = getEndPosOfPath(m_ObstacleBitMapUUDD, leftDir, y + 1, endPosX);
			if (x == -1) {
				break;
			}
		}
	}
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeLURD(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	if ((jnode.pathNode.pos.x == 0 && jnode.pathNode.pos.y == 0) || (jnode.pathNode.pos.x == m_RangeX - 1 && jnode.pathNode.pos.y == m_RangeY - 1)) {
		return;
	}

	Pos<T> coordPos = coordXYtoLURD(jnode.pathNode.pos);
	T x;
	T y = coordPos.y;

	if (leftDir) {
		if ((m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x - 1] && m_ObstacleMap[jnode.pathNode.pos.y - 1][jnode.pathNode.pos.x])
			|| (!m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x - 1] && !m_ObstacleMap[jnode.pathNode.pos.y - 1][jnode.pathNode.pos.x])) {
			return;
		}

		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y, x);
		T endOfObstacleBeside, endOfPathBeside;
		bool underObstacle;
		if (m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x - 1] && !m_ObstacleMap[jnode.pathNode.pos.y - 1][jnode.pathNode.pos.x]) {
			underObstacle = true;
		}
		else {
			underObstacle = false;
		}

		// 아래 대각선 장애물 시작
		if (underObstacle) {
			if (jnode.pathNode.pos.x == jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y + 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y - 1, x - 1);
			}
			else if (jnode.pathNode.pos.x > jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y + 1, x);
				endOfObstacleBeside -= 1;
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y - 1, x - 1);
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y + 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y - 1, x);
				endOfPathBeside -= 1;
			}
		}
		// 위 대각선 장애물 시작
		else {
			if (jnode.pathNode.pos.x == jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y - 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y + 1, x - 1);
			}
			else if (jnode.pathNode.pos.x > jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y - 1,  x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y + 1, x);
				endOfPathBeside -= 1;
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y - 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y + 1, x - 1);
				endOfObstacleBeside -= 1;
			}
		}

		if(endOfObstacleBeside < endOfPath || endOfObstacleBeside < endOfPathBeside) {
			return;
		}
		else {
			Pos<T> rpos = coordLURDtoXY({ y, endOfObstacleBeside + 1 });
			PathNode<T> rpathNode;
			rpathNode.pos = rpos;
			rpathNode.g = calculate_G(rpathNode, jnode.pathNode);
			rpathNode.h = calculate_H(rpathNode.pos, destPos);
			rpathNode.f = rpathNode.g + rpathNode.h;
			rpathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ rpathNode, UL });
#if defined DEBUG_MODE2
			cout << "[new node] y: " << rpathNode.pos.y << ", x: " << rpathNode.pos.x << ", dir: UL" << endl;
#endif

			if (underObstacle) {
				if (!m_ObstacleMap[rpos.y + 1][rpos.x - 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y + 1 , rpos.x - 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, DL });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DL" << endl;
#endif
				}
			}
			else {
				if (!m_ObstacleMap[rpos.y - 1][rpos.x + 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y - 1 , rpos.x + 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, UR });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UR" << endl;
#endif
				}
			}
		}
	}
	else {
		if ((m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x + 1] && m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x])
			|| (!m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x + 1] && !m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x])) {
			return;
		}

		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y, x);
		T endOfObstacleBeside, endOfPathBeside;
		bool underObstacle;
		if (m_ObstacleMap[jnode.pathNode.pos.y][jnode.pathNode.pos.x + 1] && !m_ObstacleMap[jnode.pathNode.pos.y + 1][jnode.pathNode.pos.x]) {
			underObstacle = false;
		}
		else {
			underObstacle = true;
		}

		// 아래 대각선 장애물 시작
		if (underObstacle) {
			if (jnode.pathNode.pos.x == jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y + 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y - 1, x);
			}
			else if (jnode.pathNode.pos.x > jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y + 1, x + 1);
				endOfObstacleBeside -= 1;
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y - 1, x);
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y + 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y - 1, x + 1);
				endOfPathBeside -= 1;
			}
		}
		else {
			if (jnode.pathNode.pos.x == jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y - 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y + 1, x - 1);
			}
			else if (jnode.pathNode.pos.x > jnode.pathNode.pos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y - 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y + 1, x);
				endOfPathBeside -= 1;
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLURD, leftDir, y - 1, x + 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLURD, leftDir, y + 1, x);
				endOfObstacleBeside -= 1;
			}
		}

		if (endOfObstacleBeside >= endOfPath || endOfObstacleBeside > endOfPathBeside) {
			return;
		}
		else {
			Pos<T> rpos = coordLURDtoXY({ y, endOfObstacleBeside });
			PathNode<T> rpathNode;
			rpathNode.pos = rpos;
			rpathNode.g = calculate_G(rpathNode, jnode.pathNode);
			rpathNode.h = calculate_H(rpathNode.pos, destPos);
			rpathNode.f = rpathNode.g + rpathNode.h;
			rpathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ rpathNode, DR });
#if defined DEBUG_MODE2
			cout << "[new node] y: " << rpathNode.pos.y << ", x: " << rpathNode.pos.x << ", dir: DR" << endl;
#endif
			
			if (underObstacle) {
				if (!m_ObstacleMap[rpos.y + 1][rpos.x - 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y + 1 , rpos.x - 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, DL });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DL" << endl;
#endif
				}
			}
			else {
				if (!m_ObstacleMap[rpos.y - 1][rpos.x + 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y - 1 , rpos.x + 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, UR });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UR" << endl;
#endif
				}
			}
		}
	}
}

template<typename T>
inline void JPSPathFinder<T>::createNewNodeLDRU(const JPSNode& jnode, bool leftDir, std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>>& openList, Pos<T> destPos, std::map<Pos<T>, Pos<T>>& parentPosMap, std::vector<PathNode<T>>& trackList)
{
	Pos<T> fixedCoordPos = coordXYtoUUDD(jnode.pathNode.pos);

	if ((fixedCoordPos.x == 0 && fixedCoordPos.y == 0) || (fixedCoordPos.x == m_RangeY - 1 && fixedCoordPos.y == m_RangeX - 1)) {
		return;
	}

	Pos<T> coordPos = coordXYtoLURD(fixedCoordPos, true);
	T x;
	T y = coordPos.y;

	if (leftDir) {
		if ((m_ObstacleMapUUDD[fixedCoordPos.y][fixedCoordPos.x - 1] && m_ObstacleMapUUDD[fixedCoordPos.y - 1][fixedCoordPos.x])
			|| (!m_ObstacleMapUUDD[fixedCoordPos.y][fixedCoordPos.x - 1] && !m_ObstacleMapUUDD[fixedCoordPos.y - 1][fixedCoordPos.x])) {
			return;
		}

		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y, x);
		T endOfObstacleBeside, endOfPathBeside;
		bool underObstacle;
		if (m_ObstacleMapUUDD[fixedCoordPos.y][fixedCoordPos.x - 1] && !m_ObstacleMapUUDD[fixedCoordPos.y - 1][fixedCoordPos.x]) {
			underObstacle = true;
		}
		else {
			underObstacle = false;
		}

		// 아래 대각선 장애물 시작
		if (underObstacle) {
			if (fixedCoordPos.x == fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y + 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y - 1, x - 1);
			}
			else if (fixedCoordPos.x > fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y + 1, x);
				endOfObstacleBeside -= 1;
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y - 1, x - 1);
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y + 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y - 1, x);
				endOfPathBeside -= 1;
			}
		}
		// 위 대각선 장애물 시작
		else {
			if (fixedCoordPos.x == fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y - 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y + 1, x - 1);
			}
			else if (fixedCoordPos.x > fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y - 1, x - 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y + 1, x);
				endOfPathBeside -= 1;
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y - 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y + 1, x - 1);
				endOfObstacleBeside -= 1;
			}
		}

		if (endOfObstacleBeside < endOfPath || endOfObstacleBeside < endOfPathBeside) {
			return;
		}
		else {
			Pos<T> rpos = coordLURDtoXY({ y, endOfObstacleBeside + 1 }, true);
			rpos = coordUUDDtoXY(rpos);
			PathNode<T> rpathNode;
			rpathNode.pos = rpos;
			rpathNode.g = calculate_G(rpathNode, jnode.pathNode);
			rpathNode.h = calculate_H(rpathNode.pos, destPos);
			rpathNode.f = rpathNode.g + rpathNode.h;
			rpathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ rpathNode, DL });
#if defined DEBUG_MODE2
			cout << "[new node] y: " << rpathNode.pos.y << ", x: " << rpathNode.pos.x << ", dir: DL" << endl;
#endif

			if (underObstacle) {
				if (!m_ObstacleMap[rpos.y + 1][rpos.x + 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y + 1 , rpos.x + 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, DR });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DR" << endl;
#endif
				}
			}
			else {
				if (!m_ObstacleMap[rpos.y - 1][rpos.x - 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y - 1 , rpos.x - 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, UL });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UL" << endl;
#endif
				}
			}
		}
	}
	else {
		if ((m_ObstacleMapUUDD[fixedCoordPos.y][fixedCoordPos.x + 1] && m_ObstacleMapUUDD[fixedCoordPos.y + 1][fixedCoordPos.x])
			|| (!m_ObstacleMapUUDD[fixedCoordPos.y][fixedCoordPos.x + 1] && !m_ObstacleMapUUDD[fixedCoordPos.y + 1][fixedCoordPos.x])) {
			return;
		}

		x = coordPos.x;
		T endOfPath = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y, x);
		T endOfObstacleBeside, endOfPathBeside;
		bool underObstacle;
		if (m_ObstacleMapUUDD[fixedCoordPos.y][fixedCoordPos.x + 1] && !m_ObstacleMapUUDD[fixedCoordPos.y + 1][fixedCoordPos.x]) {
			underObstacle = false;
		}
		else {
			underObstacle = true;
		}

		// 아래 대각선 장애물 시작
		if (underObstacle) {
			if (fixedCoordPos.x == fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y + 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y - 1, x);
			}
			else if (fixedCoordPos.x > fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y + 1, x + 1);
				endOfObstacleBeside -= 1;
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y - 1, x);
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y + 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y - 1, x + 1);
				endOfPathBeside -= 1;
			}
		}
		else {
			if (fixedCoordPos.x == fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y - 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y + 1, x - 1);
			}
			else if (fixedCoordPos.x > fixedCoordPos.y) {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y - 1, x);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y + 1, x);
				endOfPathBeside -= 1;
			}
			else {
				endOfObstacleBeside = getEndPosOfObstacle(m_ObstacleBitMapLDRU, leftDir, y - 1, x + 1);
				endOfPathBeside = getEndPosOfPath(m_ObstacleBitMapLDRU, leftDir, y + 1, x);
				endOfObstacleBeside -= 1;
			}
		}

		if (endOfObstacleBeside >= endOfPath || endOfObstacleBeside > endOfPathBeside) {
			return;
		}
		else {
			Pos<T> rpos = coordLURDtoXY({ y, endOfObstacleBeside }, true);
			rpos = coordUUDDtoXY(rpos);
			PathNode<T> rpathNode;
			rpathNode.pos = rpos;
			rpathNode.g = calculate_G(rpathNode, jnode.pathNode);
			rpathNode.h = calculate_H(rpathNode.pos, destPos);
			rpathNode.f = rpathNode.g + rpathNode.h;
			rpathNode.parentPos = jnode.pathNode.pos;
			openList.push(JPSNode{ rpathNode, UR });
#if defined DEBUG_MODE2
			cout << "[new node] y: " << rpathNode.pos.y << ", x: " << rpathNode.pos.x << ", dir: UR" << endl;
#endif

			if (underObstacle) {
				if (!m_ObstacleMap[rpos.y + 1][rpos.x + 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y + 1 , rpos.x + 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, DR });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: DR" << endl;
#endif
				}
			}
			else {
				if (!m_ObstacleMap[rpos.y - 1][rpos.x - 1]) {
					PathNode<T> pathNode;
					pathNode.pos = { rpos.y - 1 , rpos.x - 1 };
					pathNode.g = calculate_G(pathNode, rpathNode);
					pathNode.h = calculate_H(pathNode.pos, destPos);
					pathNode.f = pathNode.g + pathNode.h;
					pathNode.parentPos = rpathNode.pos;
					openList.push(JPSNode{ pathNode, UL });
#if defined DEBUG_MODE2
					cout << "[new node] y: " << pathNode.pos.y << ", x: " << pathNode.pos.x << ", dir: UL" << endl;
#endif
				}
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
	//return Pos<T>{m_RangeX - 1 - pos.x, pos.y};
	return Pos<T>{pos.x, m_RangeY - 1 - pos.y};
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordXYtoLURD(Pos<T> pos, bool axisTransform)
{
	T Range;
	if (axisTransform) {
		Range = m_RangeY;
	}
	else {
		Range = m_RangeX;
	}

	Pos<T> coordPos;
	coordPos.y = Range - 1 - pos.x + pos.y;
	if (pos.x >= pos.y) {
		coordPos.x = pos.y;
	}
	else {
		coordPos.x = pos.x;
	}

	return coordPos;
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordXYtoLDRU(Pos<T> pos, bool axisTransform)
{
	T Range;
	if (axisTransform) {
		Range = m_RangeX;
	}
	else {
		Range = m_RangeY;
	}

	T ry = pos.x;
	T rx = Range - 1 - pos.y;

	Pos<T> coordPos;
	coordPos.y = Range - 1 - rx + ry;
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
	//return Pos<T>{m_RangeX - 1 - pos.x, pos.y};
	// nY = m_RangeX - 1 - x
	// nX = y;
	//return Pos<T>{pos.x, m_RangeX - 1 - pos.y};

	
	//return Pos<T>{pos.x, m_RangeY - 1 - pos.y};
	// nY = x
	// nX = m_Range_Y - 1 - pos.y
	// y = m_Range_Y - 1 - nX
	return Pos<T>{m_RangeY- 1 - pos.x, pos.y};
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordLURDtoXY(Pos<T> pos, bool axisTransform)
{
	T Range;
	if (axisTransform) {
		Range = m_RangeY;
	}
	else {
		Range = m_RangeX;
	}

	Pos<T> coordPos;
	if (Range - 1 - pos.y >= 0) {		// x >= y
		// nX = y;
		coordPos.y = pos.x;
		coordPos.x = coordPos.y + Range - 1 - pos.y;
	}
	else {							
		// nX = x;
		coordPos.x = pos.x;
		coordPos.y = coordPos.x - Range + 1 + pos.y;
	}
	return coordPos;
}

template<typename T>
inline Pos<T> JPSPathFinder<T>::coordLDRUtoXY(Pos<T> pos, bool axisTransform)
{
	T Range;
	if (axisTransform) {
		Range = m_RangeX;
	}
	else {
		Range = m_RangeY;
	}
	
	Pos<T> coordPos;
	T ry, rx;

	if (Range - 1 - pos.y >= 0) {
		ry = pos.x;
		rx = Range - 1 - pos.y + ry;
	}
	else {
		rx = pos.x;
		ry = rx - Range + 1 + pos.y;
	}

	coordPos.x = ry;
	coordPos.y = Range - 1 - rx;

	return coordPos;
}
