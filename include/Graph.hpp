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
#include <queue>
#include <algorithm>
#include <limits>
#include <unordered_set>

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

	inline const T & getDestinationNode() const { return m_destinationNode; }
	inline const double & getCost() const { return m_cost; }
private:
	T m_destinationNode;
	double m_cost;
};

template<class T>
class Graph
{
	using myPair = typename std::pair<double, T>;
	using myHeap = typename std::priority_queue<myPair, std::vector<myPair>, Compare<T>>;

public:
	Graph();
	~Graph();

	void addEdge(const T & i_v, const T & i_w);
	void addEdge(const T & i_v, const T & i_w, const double i_cost);

    inline const unsigned long getNumberOfNodes() const { return m_graph.size(); }

	double Dijkstra(const T & i_source, const T & i_target, std::vector<T> & i_path);
	double BidirDijkstra(const T & i_source, const T & i_target, std::vector<T> & i_path);
    void printPath(const std::vector<T> & i_path);

private:
	//adjacency list representation
	std::unordered_map<T, std::vector<Edge<T>>> m_graph;

	void traceBackPath(const T & i_source, const T & i_target, std::unordered_map<T, T> & i_parents, std::vector<T> & i_path);
	void bidirtraceBackPath(const T & i_source, const T & i_target, const T & i_touchNode, std::unordered_map<T, T> & i_parentsF, std::unordered_map<T, T> & i_parentsB, std::vector<T> & i_path);
	void expandForwardFrontier(myHeap & i_queueF, std::unordered_set<T> & i_closedF, std::unordered_map<T, double> & i_distF, std::unordered_map<T, T> & i_parentsF, 
							   std::unordered_set<T> & i_closedB, std::unordered_map<T, double> & i_distB, double & i_bestPathLength, T & touchNode);
	void expandBackwardFrontier(myHeap & i_queueB, std::unordered_set<T> & i_closedB, std::unordered_map<T, double> & i_distB, std::unordered_map<T, T> & i_parentsB, 
								std::unordered_set<T> & i_closedF, std::unordered_map<T, double> & i_distF, double & i_bestPathLength, T & touchNode);
	void updateForwardFrontier(const T & i_node, const double i_cost, std::unordered_set<T> & i_closedB, std::unordered_map<T, double> & i_distB, double & i_bestPathLength, T & touchNode);
	void updateBackwardFrontier(const T & i_node, const double i_cost, std::unordered_set<T> & i_closedF, std::unordered_map<T, double> & i_distF, double & i_bestPathLength, T & touchNode);
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

template<class T>
void Graph<T>::addEdge(const T& i_v, const T& i_w)
{
	try
	{
		m_graph[i_v].push_back(Edge<T>(i_w));
		m_graph[i_w].push_back(Edge<T>(i_v));
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
		m_graph[i_v].push_back(Edge<T>(i_w, i_cost));
		m_graph[i_w].push_back(Edge<T>(i_v, i_cost));
	}
	catch(std::exception & e)
	{
			throw e;
	}
}

template<class T>
double Graph<T>::Dijkstra(const T& i_source, const T& i_target, std::vector<T> & i_path)
{
	try
	{
		myHeap queue;
		std::unordered_map<T, double> dist;
		std::unordered_map<T, T> parents;

		queue.push(std::make_pair(0, i_source));
		dist[i_source] = 0;

		while (!queue.empty())
		{
			T u = queue.top().second;
			queue.pop();

			if (u == i_target)
			{
				traceBackPath(i_source, i_target, parents, i_path);
				return dist[i_target];
			}

			std::vector<Edge<T>>::const_iterator it;
			for (it = m_graph[u].begin(); it != m_graph[u].end(); ++it)
			{
			T v = it->getDestinationNode();
			double cost = it->getCost();

				if (dist.find(v) == dist.end() || dist[v] > dist[u] + cost)
				{
					dist[v] = dist[u] + cost;
					parents[v] = u;
					queue.push(std::make_pair(dist[v], v));
				}
			}
		}

		return -1;
	}
	catch(std::exception & e)
	{
		throw e;
	}
}

template<class T>
inline double Graph<T>::BidirDijkstra(const T & i_source, const T & i_target, std::vector<T>& i_path)
{
	try
	{
		myHeap queueF;
		myHeap queueB;
		std::unordered_set<T> closedF, closedB;
		std::unordered_map<T, double> distF, distB;
		std::unordered_map<T, T> parentsF, parentsB;

		queueF.push(std::make_pair(0, i_source));
		queueB.push(std::make_pair(0, i_target));
		double bestPathLength = std::numeric_limits<double>::max();
		distF[i_source] = 0.0;
		distB[i_target] = 0.0;

		T touchNode;

		while (!queueF.empty() && !queueB.empty())
		{
			double tmp = distF[queueF.top().second] + distB[queueB.top().second];

			if (tmp >= bestPathLength)
			{
				bidirtraceBackPath(i_source, i_target, touchNode, parentsF, parentsB, i_path);
				return bestPathLength;
			}

			if ((queueF.size() + closedF.size()) < (queueB.size() + closedB.size()))
				expandForwardFrontier(queueF, closedF, distF, parentsF, closedB, distB, bestPathLength, touchNode);
			else
				expandBackwardFrontier(queueB, closedB, distB, parentsB, closedF, distF, bestPathLength, touchNode);
		}

		return -1;
	}
	catch (std::exception & e)
	{
		throw e;
	}
}

template<class T>
void Graph<T>::traceBackPath(const T & i_source, const T & i_target, std::unordered_map<T, T> & i_parents, std::vector<T> & i_path)
{
	i_path.push_back(i_target);
	T temp = i_target;
	while (i_parents[temp] != i_source)
	{
		i_path.push_back(i_parents[temp]);
		temp = i_parents[temp];
	}
	i_path.push_back(i_source);

	std::reverse(i_path.begin(), i_path.end());
}

template<class T>
void Graph<T>::bidirtraceBackPath(const T & i_source, const T & i_target, const T & i_touchNode, std::unordered_map<T, T> & i_parentsF, std::unordered_map<T, T> & i_parentsB, std::vector<T> & i_path)
{
	traceBackPath(i_source, i_touchNode, i_parentsF, i_path);
	T temp = i_touchNode;
	while (i_parentsB[temp] != i_target)
	{
		i_path.push_back(i_parentsB[temp]);
		temp = i_parentsB[temp];
	}
	i_path.push_back(i_target);
}

template<class T>
inline void Graph<T>::printPath(const std::vector<T>& i_path)
{
	const size_t size = i_path.size();
	std::cout << "Path: ";
	for (size_t i = 0; i < size; ++i)
	{
		if (i != size - 1)
			std::cout << i_path[i] << " --> ";
		else
			std::cout << i_path[i] << "\n";
	}
}

template<class T>
void Graph<T>::expandForwardFrontier(myHeap & i_queueF, std::unordered_set<T> & i_closedF, std::unordered_map<T, double> & i_distF, std::unordered_map<T, T> & i_parentsF,
									 std::unordered_set<T> & i_closedB, std::unordered_map<T, double> & i_distB, double & i_bestPathLength, T & i_touchNode)
{
	T u = i_queueF.top().second;
	i_closedF.insert(u);
	i_queueF.pop();

	std::vector<Edge<T>>::const_iterator it;
	for (it = m_graph[u].begin(); it != m_graph[u].end(); ++it)
	{
		T v = it->getDestinationNode();
		if (i_closedF.find(v) == i_closedF.end())
		{
			double cost = it->getCost();

			if (i_distF.find(v) == i_distF.end() || i_distF[v] > i_distF[u] + cost)
			{
				i_distF[v] = i_distF[u] + cost;
				i_parentsF[v] = u;
				i_queueF.push(std::make_pair(i_distF[v], v));
				updateForwardFrontier(v, i_distF[v], i_closedB, i_distB, i_bestPathLength, i_touchNode);
			}
		}
	}
}

template<class T>
void Graph<T>::expandBackwardFrontier(myHeap & i_queueB, std::unordered_set<T>& i_closedB, std::unordered_map<T, double>& i_distB, std::unordered_map<T, T>& i_parentsB,
									  std::unordered_set<T> & i_closedF, std::unordered_map<T, double> & i_distF, double & i_bestPathLength, T & i_touchNode)
{
	T u = i_queueB.top().second;
	i_closedB.insert(u);
	i_queueB.pop();

	std::vector<Edge<T>>::const_iterator it;
	for (it = m_graph[u].begin(); it != m_graph[u].end(); ++it)
	{
		T v = it->getDestinationNode();
		if (i_closedB.find(v) == i_closedB.end())
		{
			double cost = it->getCost();

			if (i_distB.find(v) == i_distB.end() || i_distB[v] > i_distB[u] + cost)
			{
				i_distB[v] = i_distB[u] + cost;
				i_parentsB[v] = u;
				i_queueB.push(std::make_pair(i_distB[v], v));
				updateBackwardFrontier(v, i_distB[v], i_closedF, i_distF, i_bestPathLength, i_touchNode);
			}
		}
	}
}

template<class T>
void Graph<T>::updateForwardFrontier(const T & i_node, const double i_cost, std::unordered_set<T> & i_closedB, std::unordered_map<T, double> & i_distB, double & i_bestPathLength, T & i_touchNode)
{
	if (i_closedB.find(i_node) != i_closedB.end())
	{
		double pathLen = i_distB[i_node] + i_cost;

		if (i_bestPathLength > pathLen)
		{
			i_bestPathLength = pathLen;
			i_touchNode = i_node;
		}
	}
}

template<class T>
void Graph<T>::updateBackwardFrontier(const T & i_node, const double i_cost, std::unordered_set<T> & i_closedF, std::unordered_map<T, double >& i_distF, double & i_bestPathLength, T & i_touchNode)
{
	if (i_closedF.find(i_node) != i_closedF.end())
	{
		double pathLen = i_distF[i_node] + i_cost;

		if (i_bestPathLength > pathLen)
		{
			i_bestPathLength = pathLen;
			i_touchNode = i_node;
		}
	}
}

#endif /* GRAPH_HPP_ */