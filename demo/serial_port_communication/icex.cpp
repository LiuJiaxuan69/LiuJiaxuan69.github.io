#include <iostream>
#include <fstream>
#include <string>

int cnt = 0;  // 全局计数器

// 假设这是你已经实现的提取函数
std::string extractor(const std::string& input) {
    std::string result;
    if(input.find("(replaced)") != std::string::npos) {
        return result;  // 如果没有 "(replaced)"，直接返回原始字符串
    }
    int sz = input.size();
    for(int i = 0; i <= sz - 8; ++i)
    {
        bool flag = false;
        for(int j = 0; j < 8; ++j)
        {
            if(input[i + j] >= '0' && input[i + j] <= '9') continue;
            else {
                flag = true;  // 如果有非数字字符，标记为true
                break;
            }
        }
        if(!flag) {  // 如果没有非数字字符
            result += input.substr(i, 8);  // 提取8个字符
            std::cout << "Extracted: " << result << std::endl;  // 输出提取的字符串
            ++cnt;  // 增加计数器
            return result;
        }
    }
    return ""; // 如果没有找到符合条件的字符串，返回空字符串
}

void processFile(const std::string& inputFilename) {
    // 构造输出文件名
    size_t dotPos = inputFilename.find_last_of('.');
    std::string outputFilename = inputFilename.substr(0, dotPos) + "_ic.hex";

    // 打开输入文件
    std::ifstream inputFile(inputFilename);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open input file " << inputFilename << std::endl;
        return;
    }

    // 打开输出文件
    std::ofstream outputFile(outputFilename);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Could not create output file " << outputFilename << std::endl;
        inputFile.close();
        return;
    }

    // 逐行处理文件
    std::string line;
    while (std::getline(inputFile, line)) {
        std::string result = extractor(line);
        if (!result.empty()) {  // 只有结果非空时才写入
            outputFile << result << '\n';
        }
    }

    // 关闭文件
    inputFile.close();
    outputFile.close();

    std::cout << "Processing completed. Results written to " << outputFilename << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file.xxx>" << std::endl;
        return 1;
    }
    std::string inputFilename = argv[1];
    processFile(inputFilename);
    std::ofstream outputFile(inputFilename.substr(0, inputFilename.find_last_of('.')) + "_ic.hex", std::ios::app);
    for(int i = cnt; i < 256; ++i){
        outputFile << "00000000\n";  // 填充空行
    }
    return 0;
}
