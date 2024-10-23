//
// Created by Cao Rui on 24-8-30.
//

#ifndef CUSTOM_BARRIER_H
#define CUSTOM_BARRIER_H

#include <condition_variable>

class CustomBarrier{
private:
    std::mutex mtx;
    std::condition_variable cv;
    const int num_threads;
    int count;
    int generation;
public:
    CustomBarrier() = delete;
    CustomBarrier(const CustomBarrier&) = delete;
    CustomBarrier& operator=(const CustomBarrier&) = delete;

    /**
     * @brief Construct a new Custom Barrier object with the number of threads.
     * @param n The number of threads in the group sharing the barrier object.
     */
    explicit CustomBarrier(int n);

    /**
     * @brief Wait for all threads in the group to reach the barrier.
     */
    void wait();
};

#endif //CUSTOM_BARRIER_H
