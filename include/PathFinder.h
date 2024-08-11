#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <algorithm>
#include <cassert>
#include <utility>
#include "PathStruct.h"

template<typename T>
class PathFinder
{
public:
	class iterator {
		Path<T> path;
		Pos<T>* pathPtr;
	public:
		iterator() {}
		iterator(Path<T>&& path) {
			this->path = std::move(path);
			this->pathPtr = this->path.Now();
		}
		void operator=(Path<T>&& path) {
			this->path = std::move(path);
			this->pathPtr = this->path.Now();
		}
		iterator& operator++() {
			pathPtr = path.Next();
			return *this;
		}
		Pos<T>& operator*() {
			return *pathPtr;
		}
		bool End() {
			if (pathPtr == nullptr) {
				return true;
			}
			else {
				return false;
			}
		}
	};

public:
	virtual void Init(T rangeY, T rangeX);
	virtual void SetObstacle(T y, T x);
	virtual void UnsetObstacle(T y, T x);

	virtual iterator FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList) = 0;

protected:
	T calculate_H(const Pos<T>& start, const Pos<T>& dest);
	T calculate_G(const PathNode<T>& node, const PathNode<T>& parentNode);
	bool checkAvailability(T y, T x);

private:
	T mahattan(const Pos<T>& startPos, const Pos<T>& destPos, int delta) {
		return (abs(destPos.y - startPos.y) + abs(destPos.x - startPos.x)) * delta;
	}
	T euclidean_integer(const Pos<T>& startPos, const Pos<T>& destPos, BYTE delta) {
		double distance;
		distance = sqrt(pow(startPos.x - destPos.x, 2) + pow(startPos.y - destPos.y, 2));
		return lround(distance * delta);
	}

protected:
	T m_RangeY;
	T m_RangeX;
	std::vector<std::vector<bool>> m_ObstacleMap;
	BYTE* m_ChunkLLRR;											// LL RR
	std::vector<std::vector<BYTE*>> m_ObstacleBitMapLLRR;		// LL RR 
};

template<typename T>
void PathFinder<T>::Init(T rangeY, T rangeX)
{
	m_RangeY = rangeY;
	m_RangeX = rangeX;

	int colLength;

	// LL RR
	colLength = m_RangeX / BYTE_BIT + ((m_RangeX % BYTE_BIT == 0) ? 0 : 1);
	m_ChunkLLRR = new BYTE[m_RangeY * colLength];
	memset(m_ChunkLLRR, 0, m_RangeY * colLength);
	m_ObstacleMap.resize(m_RangeY);
	m_ObstacleBitMapLLRR.resize(m_RangeY);
	for (int y = 0; y < m_RangeY; y++) {
		m_ObstacleMap[y].resize(m_RangeX, false);
		m_ObstacleBitMapLLRR[y].resize(colLength);
		for (int x = 0; x < colLength; x++) {
			m_ObstacleBitMapLLRR[y][x] = m_ChunkLLRR + y * colLength + x;
		}
	}
	if (m_RangeX % BYTE_BIT != 0) {
		int resBit = m_RangeX % BYTE_BIT;
		BYTE mask = 0b1111'1111;
		mask = mask >> resBit;

		for (int y = 0; y < m_RangeY; y++) {
			*m_ObstacleBitMapLLRR[y][colLength - 1] = *m_ObstacleBitMapLLRR[y][colLength - 1] | mask;
		}
	}
}

template<typename T>
void PathFinder<T>::SetObstacle(T y, T x)
{
	m_ObstacleMap[y][x] = true;
	*m_ObstacleBitMapLLRR[y][x / BYTE_BIT] = *m_ObstacleBitMapLLRR[y][x / BYTE_BIT] | setMasks[x % BYTE_BIT];
}

template<typename T>
void PathFinder<T>::UnsetObstacle(T y, T x)
{
	m_ObstacleMap[y][x] = false;
	*m_ObstacleBitMapLLRR[y][x / BYTE_BIT] = *m_ObstacleBitMapLLRR[y][x / BYTE_BIT] & unsetMasks[x % BYTE_BIT];
}

template<typename T>
typename PathFinder<T>::iterator PathFinder<T>::FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList)
{
	return iterator();
}

template<typename T>
T PathFinder<T>::calculate_H(const Pos<T>& start, const Pos<T>& dest)
{
	return mahattan(start, dest, 10);
}

template<typename T>
T PathFinder<T>::calculate_G(const PathNode<T>& node, const PathNode<T>& parentNode)
{
	if (node.pos.y == parentNode.pos.y) {
		return parentNode.g + 10 * std::abs(node.pos.x - parentNode.pos.x);
	}
	else if (node.pos.x == parentNode.pos.x) {
		return parentNode.g + 10 * std::abs(node.pos.y - parentNode.pos.y);
	}
	else {
		return parentNode.g + 14 * std::abs(node.pos.x - parentNode.pos.x);
	}
}

template<typename T>
bool PathFinder<T>::checkAvailability(T y, T x)
{
	if (y < 0 || x < 0 || y >= m_RangeY || x >= m_RangeX) {
		return false;
	}
	else {
		if ((*(*m_ObstacleBitMapLLRR)[y][x / BYTE_BIT] & setMasks[x % BYTE_BIT]) == 0) {
			// 장애물|벽 x
			return true;
		}
		else {
			return false;
		}
	}
}
