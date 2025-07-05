#include <iostream>
#include <fstream>
#include <string>

double calculater(const std::string &inputFilename, const std::string &compare_num)
{
    std::ifstream inputFile(inputFilename);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open input file " << inputFilename << std::endl;
        return -1;
    }
    int total_num = 0, true_num = 0;
    std::string word;
    while(inputFile >> word)
    {
        ++total_num;
        if(word == compare_num) ++true_num;
    }
    return (double)(true_num) / total_num;
}


int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file.xxx>" << "compare_str" << std::endl;
        return 1;
    }
    std::string inputFilename = argv[1];
    std::cout << calculater(inputFilename, argv[2]);
    return 0;
}