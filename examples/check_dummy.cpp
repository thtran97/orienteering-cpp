#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <random>

int main() {
    std::cout << std::round(0.5) << std::endl;
    std::cout << std::round(0.412134315) << std::endl;
    std::cout << std::round(0.5745233111) << std::endl;
    std::cout << std::ceil(0.412134315) << std::endl;
    std::cout << std::ceil(0.5745233111) << std::endl;
    
    // test shuffle vector
    std::random_device rd;
    std::mt19937 rnd_gen(rd());
    std::vector<int> vec({1,2,3,4,5});
    std::shuffle(vec.begin(), vec.end(), rnd_gen);
    for (int i: vec)
        std::cout << i << " ";
    std::cout << std::endl;

    return 0;
}
