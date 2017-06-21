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
#include <queue>

template<class T>
class Compare
{
public:
	bool operator()(const std::pair<double, T> & lhs, std::pair<double, T> & rhs) { return lhs.first > rhs.first; };
};

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

	void addEdge(const T & i_v, const T & i_w);
	void addEdge(const T & i_v, const T & i_w, const double i_cost);

    inline const unsigned long getNumberOfNodes() const { return m_graph.size(); }

    double Dijkstra(const T & i_source, const T & i_target);

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
	using myIt = typename std::unordered_map<T,std::vector<std::unique_ptr<Edge<T>>>>::iterator;
	myIt itEnd = m_graph.end();
	for (myIt it = m_graph.begin(); it != itEnd; ++it)
	{
		const size_t size = it->second.size();
		for (size_t i = 0; i < size; ++i)
		{
			it->second[i].reset(nullptr);
		}
	}
}

template<class T>
void Graph<T>::addEdge(const T& i_v, const T& i_w)
{
	try
	{
			m_graph[i_v].push_back(std::make_unique<Edge<T>>(i_w));
			m_graph[i_w].push_back(std::make_unique<Edge<T>>(i_v));
	}
	catch(std::exception & e)
	{
			throw e;
	}
}

template<class T>
void Graph<T>::addEdge(const T& i_v, const T& i_w, const double i_cost)
{
	try
	{
			m_graph[i_v].push_back(std::make_unique<Edge<T>>(i_w, i_cost));
			m_graph[i_w].push_back(std::make_unique<Edge<T>>(i_v, i_cost));
	}
	catch(std::exception & e)
	{
			throw e;
	}
}

template<class T>
double Graph<T>::Dijkstra(const T& i_source, const T& i_target)
{
	try
	{
		using myPair = typename std::pair<double, T>;
		using myHeap = typename std::priority_queue<myPair, std::vector<myPair>, Compare<T>>;

		myHeap queue;
		std::unordered_map<T, double> dist;

		queue.push(std::make_pair(0, i_source));
		dist[i_source] = 0;

		while (!queue.empty())
		{
			T u = queue.top().second;
			queue.pop();

			if (u == i_target) break;

			typename std::vector<std::unique_ptr<Edge<T>>>::const_iterator it;
			for (it = m_graph[u].begin(); it != m_graph[u].end(); ++it)
			{
				T v = (*it)->getDestionationNode();
				double cost = (*it)->getCost();

				if (dist.find(v) == dist.end() || dist[v] > dist[u] + cost)
				{
					dist[v] = dist[u] + cost;
					queue.push(std::make_pair(dist[v], v));
				}
			}
		}

		return dist.find(i_target) == dist.end() ? -1 : dist[i_target];
	}
	catch(std::exception & e)
	{
		throw e;
	}
}

#endif /* GRAPH_HPP_ */
