//
// Created by Cao Rui on 24-8-30.
//

/**
 * Answer to extra questions:
 * 1. Number of threads in parallel version: 32 (world_size)
 * 2. expected improvement of performance: 32 times, as
 * each task can be run concurrently, and each task
 * has a load 1/32 of original size.
 * Actual improvement of performance: 4.88334x, much slower 
 * than expectation. Reasoning: 
 * Overheads of creating and managing threads, blocked waiting 
 * for other threads etc. restrict the speed of all threads.
 * Also, there are not as many cores to run all threads 
 * concurrently.
 * 
 * Bonus: 16 threads -> 4.043 times improvement;
 * 8 threads -> 3.50579 times improvement;
 * 4 threads -> 4.12158 times improvement;
 * 4-32 threads only give slight improvement, probably because no enough cores are availble.
 * 2 threads -> 1.77541 times improvement;
 * 1 thread -> 0.908775 times imporvement, showing overhead of parallele version can slow progress down.
 */
#include "custom_barrier.h"

#include <mutex>
#include <utils.h>

CustomBarrier::CustomBarrier(const int n):num_threads(n){
    mtx;
    cv;
    num_arrived = 0;
}

void CustomBarrier::wait(){
    std::unique_lock <std::mutex> lck(mtx);
    num_arrived++;
    if (num_arrived < num_threads) {
        cv.wait(lck);
    }
    if (num_arrived == num_threads) {
        cv.notify_all();
        num_arrived = 0;
    }
}
