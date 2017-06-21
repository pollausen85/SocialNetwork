/*
 * Graph.hpp
 *
 *  Created on: Jun 21, 2017
 *      Author: davide
 */

#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <unordered_map>
#include <vector>
#include <memory>

template<class T>
class Edge
{
public:
	Edge(const T & i_destinationNode);
	Edge(const T & i_key, const double i_cost);
	~Edge();

	inline const T & getDestionationNode() const {return m_destinationNode;}
	inline const double & getCost() const {return m_cost;}
private:
	T m_destinationNode;
	double m_cost;
};

template<class T>
class Graph
{
public:
	Graph();
	~Graph();

private:
	//adjacency list representation
	std::unordered_map<T,std::vector<std::unique_ptr<Edge<T>>>> m_graph;
};

template<class T>
Edge<T>::Edge(const T & i_destinationNode)
	: m_destinationNode(i_destinationNode)
	, m_cost(1.0)
{
}

template<class T>
Edge<T>::Edge(const T &i_destinationNode, const double i_cost)
	: m_destinationNode(i_destinationNode)
	, m_cost(i_cost)
{
}

template<class T>
Edge<T>::~Edge()
{
}

template<class T>
Graph<T>::Graph()
{
}

template<class T>
Graph<T>::~Graph()
{
}

#endif /* GRAPH_HPP_ */
