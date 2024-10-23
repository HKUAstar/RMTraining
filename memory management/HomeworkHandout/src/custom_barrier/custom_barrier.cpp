//
// Created by Cao Rui on 24-8-30.
//

#include "custom_barrier.h"

#include <mutex>
#include <utils.h>

CustomBarrier::CustomBarrier(int num_threads)
    : num_threads(num_threads), count(0), generation(0) {}

void CustomBarrier::wait() {
    std::unique_lock<std::mutex> lock(mtx);
    int gen = generation;

    if (++count == num_threads) {
        generation++;
        count = 0;
        cv.notify_all();
    } else {
        cv.wait(lock, [this, gen] { return gen != generation; });
    }
}