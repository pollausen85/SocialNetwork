#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "Graph.hpp"
#include <ctime>

int main(int argc, char *argv[])
{
	try
	{
		if (argc != 4)
		{
			std::cout << "Wrong number of arguments" << "\n";
			return 0;
		}

		Graph<std::string> socialNetwork;

		std::ifstream file(argv[1]);
		std::string line;
		while(std::getline(file, line))
		{
			std::istringstream iss(line);
			std::string user1, user2;
			if(std::getline(iss, user1,',') && std::getline(iss,user2))
			{
				socialNetwork.addEdge(user1,user2);
			}
		}

		std::cout << "Total number of people in the Social Network: " << socialNetwork.getNumberOfNodes() << "\n";

		std::vector<std::string> pathDijkstra, pathBidirDijkstra;

		std::clock_t begin = clock();
		double minDistDijkstra = socialNetwork.Dijkstra(argv[2], argv[3], pathDijkstra);
		double elapsed_secs = double(clock() - begin) / CLOCKS_PER_SEC;
		if (-1 == minDistDijkstra)
			std::cout << "No path available between source and target" << "\n";
		else
		{
			std::cout << "Minimum distance: " << minDistDijkstra << " (computed wiht Dijkstra in: " << elapsed_secs << " s)\n";
			socialNetwork.printPath(pathDijkstra);
		}

		begin = clock();
		double minDistBidirDijkstra = socialNetwork.BidirDijkstra(argv[2], argv[3], pathBidirDijkstra);
		elapsed_secs = double(clock() - begin) / CLOCKS_PER_SEC;
		if (-1 == minDistBidirDijkstra)
			std::cout << "No path available between source and target" << "\n";
		else
		{
			std::cout << "Minimum distance: " << minDistBidirDijkstra << " (computed with Bidirectional Dijkstra in: " << elapsed_secs << " s)\n";
			socialNetwork.printPath(pathBidirDijkstra);
		}

		return 0;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << "\n";
	}
}
