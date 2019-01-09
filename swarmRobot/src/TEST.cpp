//
// Created by smanier on 18.10.18.
//

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <vector>
#include <algorithm>

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

int main(int argv, char **argc) {
    std::vector<int> a = {1, 2, 3, 4, 5, 6};

    while (a.size() > 0) {
        int num;
        std::cin >> num;

        auto pos = std::remove_if(a.begin(), a.end(), [num](int i) {
            return i < num;
        });
        std::for_each(a.begin(), a.end(), [](int i) {
            std::cout << i << " ";
        });
        std::cout << std::endl;

        a.erase(pos, a.end());
        std::for_each(a.begin(), a.end(), [](int i) {
            std::cout << i << " ";
        });
        std::cout << std::endl;
    }

    return 0;
}