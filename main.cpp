#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

int main(int argc, char *argv[])
{
	std::ifstream file(argv[1]);
	std::string line;
	while(std::getline(file, line))
	{
		std::istringstream iss(line);
		std::string user1, user2;
		if(std::getline(iss, user1,',') && std::getline(iss,user2))
		{
			std::cout << user1 << " " << user2 << "\n";
		}
	}

	return 0;
}
