//
// Created by Cao Rui on 24-8-30.
//

#include "custom_barrier.h"

#include <mutex>
#include <utils.h>

CustomBarrier::CustomBarrier(int num_threads):num_threads(num_threads), count(0), current_count(0){}


void CustomBarrier::wait(){
    std::unique_lock<std::mutex> lock(mtx);
    int current = current_count;
    count++;
    if (count == num_threads) {
        current_count++;
        count = 0;
        cv.notify_all();
    } else {
        cv.wait(lock, [this, current] { return current != current_count; });
    }
}
