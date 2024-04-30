#pragma once

#include "PathFinder.h"

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
			return pnode < other.pnode;
		}
		bool operator>(const JPSNode& other) const {
			return pnode > other.pnode;
		}

		PathNode<T> pnode;
		DIR parentDir;
	};
public:
	virtual void Init(T rangeY, T rangeX) override;
	virtual void SetObstacle(T y, T x) override;
	virtual void UnsetObstacle(T y, T x) override;

	virtual typename PathFinder<T>::iterator FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList) override;

private:
	T getEndPosOfPath(std::vector<std::vector<BYTE*>>& straightPath, bool leftDir, T y, T x);
	bool checkDestination(const JPSNode& jnode, Pos<T> destPos);

protected:
	using PathFinder<T>::calculate_H;
	using PathFinder<T>::checkAvailability;

protected:
	using PathFinder<T>::m_RangeY;
	using PathFinder<T>::m_RangeX;
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

	T ry = m_RangeX - 1 - x;
	T rx = y;

	// UU DD
	*m_ObstacleBitMapUUDD[ry][rx / BYTE_BIT] = *m_ObstacleBitMapUUDD[ry][rx / BYTE_BIT] | bitMask[rx % BYTE_BIT];

	T ty, tx;
	// LU RD		
	if (x >= y) {
		ty = (m_RangeX - 1) - x + y;
		tx = y;
	}
	else {
		ty = (m_RangeX - 1) + y - x;
		tx = x;
	}
	*m_ObstacleBitMapLURD[ty][tx / BYTE_BIT] = *m_ObstacleBitMapLURD[ty][tx / BYTE_BIT] | bitMask[tx % BYTE_BIT];

	// LD RU
	ry = x;
	rx = m_RangeY - 1 - y;
	if (rx >= ry) {
		ty = (m_RangeY - 1) - rx + ry;
		tx = ry;
	}
	else {
		ty = (m_RangeY - 1) + ry - rx;
		tx = rx;
	}
	*m_ObstacleBitMapLDRU[ty][tx / BYTE_BIT] = *m_ObstacleBitMapLDRU[ty][tx / BYTE_BIT] | bitMask[tx % BYTE_BIT];
}

template<typename T>
void JPSPathFinder<T>::UnsetObstacle(T y, T x)
{
	T ry = m_RangeX - 1 - x;
	T rx = y;

	// UU DD
	*m_ObstacleBitMapUUDD[ry][rx / BYTE_BIT] = *m_ObstacleBitMapUUDD[ry][rx / BYTE_BIT] & reverseMask[rx % BYTE_BIT];

	T ty, tx;
	// LU RD		
	if (x >= y) {
		ty = (m_RangeX - 1) - x + y;
		tx = y;
	}
	else {
		ty = (m_RangeX - 1) + y - x;
		tx = x;
	}
	*m_ObstacleBitMapLURD[ty][tx / BYTE_BIT] = *m_ObstacleBitMapLURD[ty][tx / BYTE_BIT] & reverseMask[tx % BYTE_BIT];

	// LD RU
	ry = x;
	rx = m_RangeY - 1 - y;
	if (rx >= ry) {
		ty = (m_RangeY - 1) - rx + ry;
		tx = ry;
	}
	else {
		ty = (m_RangeY - 1) + ry - rx;
		tx = rx;
	}
	*m_ObstacleBitMapLDRU[ty][tx / BYTE_BIT] = *m_ObstacleBitMapLDRU[ty][tx / BYTE_BIT] & reverseMask[tx % BYTE_BIT];
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
	openList.push({ startNode, NONE_DIR });

	// 탐색
	while (!openList.empty()) {
		JPSNode jnode = openList.top();
		openList.pop();
		trackList.push_back(jnode.pnode);
		Pos<T> pos = jnode.pnode.pos;
		parentPosMap.insert({ pos, jnode.pnode.parentPos });

		// 직선 또는 대각선 방향으로 도착지가 있는 지 확인
		if (checkDestination(jnode, destPos)) {
			arriveFlag = true;
			destNode.parentPos = pos;
			destNode.pos = destPos;
			parentPosMap.insert({ destPos, destNode.parentPos });
			std::cout << "도착!" << std::endl;
			break;
		}

		// 탐색 가능한 방향으로 탐색
		// LL 탐색
		if (jnode.parentDir != RR && jnode.parentDir != UR && jnode.parentDir != DR) {
			if (checkAvailability(pos.y, pos.x - 1)) {
				checkNewNode(jnode, LL, openList, destPos, parentPosMap, trackList);
			}
		}
		// RR 탐색
		if (jnode.parentDir != LL && jnode.parentDir != UL && jnode.parentDir != DL) {
			if (checkAvailability(pos.y, pos.x + 1)) {
				checkNewNode(jnode, RR, openList, destPos, parentPosMap, trackList);
			}
		}
		// UU 탐색
		if (jnode.parentDir != DD && jnode.parentDir != DL && jnode.parentDir != DR) {
			if (checkAvailability(pos.y - 1, pos.x)) {
				checkNewNode(jnode, UU, openList, destPos, parentPosMap, trackList);
			}
		}
		// DD 탐색
		if (jnode.parentDir != UU && jnode.parentDir != UL && jnode.parentDir != UR) {
			if (checkAvailability(pos.y + 1, pos.x)) {
				checkNewNode(jnode, DD, openList, destPos, parentPosMap, trackList);
			}
		}
		if (/*jnode.parentDir != UL &&*/ jnode.parentDir != DR) {
			// UL 탐색
			if (checkAvailability(pos.y - 1, pos.x - 1)) {
				checkNewNode(jnode, UL, openList, destPos, parentPosMap, trackList);
			}
		}
		if (jnode.parentDir != UL /* && jnode.parentDir != DR*/) {
			// DR 탐색
			if (checkAvailability(pos.y + 1, pos.x + 1)) {
				checkNewNode(jnode, DR, openList, destPos, parentPosMap, trackList);
			}
		}
		if (/*jnode.parentDir != UR && */jnode.parentDir != DL) {
			// UR 탐색
			if (checkAvailability(pos.y - 1, pos.x + 1)) {
				checkNewNode(jnode, UR, openList, destPos, parentPosMap, trackList);
			}
		}
		if (jnode.parentDir != UR/* && jnode.parentDir != DL*/) {
			// DL 탐색
			if (checkAvailability(pos.y + 1, pos.x - 1)) {
				checkNewNode(jnode, DL, openList, destPos, parentPosMap, trackList);
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
T JPSPathFinder<T>::getEndPosOfPath(std::vector<std::vector<BYTE*>>& straightPath, bool leftDir, T y, T x)
{
	int byteIdx = x / BYTE_BIT;
	BYTE bitIdx = x & BYTE_BIT;
	BYTE byte = *straightPath[y][x];

	T ret = 0;

	if (leftDir) {	// 좌측 직선 방향
		// x 위치의 비트 인덱스가 포함된 바이트와 해당 바이트의 MSB 체크
		if (byte != 0 && GetMSBIdx(byte) < bitIdx) {
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
bool JPSPathFinder<T>::checkDestination(const JPSNode& jnode, Pos<T> destPos)
{
	// 8방향의 직선 경로 확인


	return false;
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
	return coordXYtoLURD(Pos<T>{ry, rx});
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
	Pos<T> rPos = coordLURDtoXY(pos);
	// ry = x;
	// rx = m_RangeY - 1 - y;
	return Pos<T>{m_RangeY - 1 - rPos.x, rPos.y};
}
