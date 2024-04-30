#pragma once

#include "PathFinder.h"

template<typename T>
class AStarPathFinder : public PathFinder<T> 
{
public:
	virtual typename PathFinder<T>::iterator FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList) override;
};

template<typename T>
typename PathFinder<T>::iterator AStarPathFinder<T>::FindPath(T startY, T startX, T destY, T destX, std::vector<PathNode<T>>& trackList)
{
	return typename PathFinder<T>::iterator();
}
