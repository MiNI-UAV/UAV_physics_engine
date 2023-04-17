#include <iostream>
#include <thread>
#include <chrono>
#include "timed_loop.hpp"


int main()
{
    const int N = 1000;
    const int period = 5;
    int counter = 0;
    TimedLoop loop(5, [&counter](){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return counter++ < 2000;
    });

    auto start = std::chrono::high_resolution_clock::now();

    loop.go(N);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "Expected: " << N*period << "ms\n";
    std::cout << "Actual: "<< std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() << "ms\n";

    start = std::chrono::high_resolution_clock::now();
    loop.go();
    finish = std::chrono::high_resolution_clock::now();
    std::cout << "Expected: " << N*period << "ms\n";
    std::cout << "Actual: "<< std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() << "ms\n";
}