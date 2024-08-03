#include "gtest/gtest.h"
#include <set>
#include <list>
#include <vector>
#include <chrono>
#include <algorithm>
#include <random>
#include <iostream>

using namespace std;

TEST(setTest, testCheck){
    int const N = 1000;
    set<int> x;
    for (int i = 0; i < N; i++)
    {
        x.insert(i*2);
    }
    int nbOK = 0;
    auto t1 = chrono::high_resolution_clock::now();
    for (int i = 0; i < 2*N; i++){
        if (x.find(i) != x.end())
        {
            nbOK++;
        }
    }
    auto t2 = chrono::high_resolution_clock::now();
    ASSERT_EQ(nbOK, N);
    chrono::duration<double, std::milli> tExec = t2 - t1;
    cout << "set check: " << tExec.count() << " ms" << endl;
}
