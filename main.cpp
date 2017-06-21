#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "Graph.hpp"

int main(int argc, char *argv[])
{
        try
        {
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

                return 0;
        }
        catch(std::exception & e)
        {
                std::cout << e.what() << "\n";
        }
}

