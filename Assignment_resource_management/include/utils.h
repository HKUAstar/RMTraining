//
// Created by Cao Rui on 24-9-4.
//

#ifndef UTILS_H
#define UTILS_H

#include <chrono>

namespace utils{
    // Benchmark a function, taking
    template<typename T, typename ... Args>
    std::chrono::duration<double> benchmark(
        const int n_repeat,
        const int n_warmup,
        T&& func,
        Args&& ... args
    ){
        for(int i = 0; i < n_warmup; i++){
            std::forward<T>(func)(std::forward<Args>(args)...);
        }

        auto total_time = std::chrono::duration<double>::zero();
        for(int i = 0; i < n_repeat; i++){
            const auto start = std::chrono::high_resolution_clock::now();
            std::forward<T>(func)(std::forward<Args>(args)...);
            const auto end = std::chrono::high_resolution_clock::now();
            total_time += end - start;
        }
        return total_time / n_repeat;
    }

    inline void no_impl(){
        throw std::runtime_error("Not implemented yet.");
    }
}

#endif //UTILS_H
