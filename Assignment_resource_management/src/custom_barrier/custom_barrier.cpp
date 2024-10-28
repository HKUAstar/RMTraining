//
// Created by Cao Rui on 24-8-30.
//

#include "custom_barrier.h"

#include <mutex>
#include <condition_variable>
#include <utils.h>
#include <iostream>
#include <thread>
//when a member variable is declared as const, it must be initialized in the constructor's initializer list.
CustomBarrier::CustomBarrier(const int n):num_threads(n), count(0), generation(0){}

//block the current thread until all threads reach the barrier
void CustomBarrier::wait() {
        std::unique_lock<std::mutex> lock(this->mtx);
        int gen = this->generation;

        this->count += 1;
        if (this->count == this->num_threads) {
            this->generation += 1;
            this->count = 0;
            cv.notify_all();
        }else{
            cv.wait(lock, [this, gen] { return gen != generation; });
        }
    }

// The problem still exist in how to reset the barrier after all thread reach the barrier
// and some of them reuse the barrier immediately
// Need a variable that can only be modified by the thread which call cv.notify_all().
// Lock? No! Just use (int)generation and increment it when this->count == this->num_threads!!!