//
// Created by Cao Rui on 24-9-8.
//

#include <barrier>

#include "safe_hashmap.h"
#include "utils.h"
#include <thread>
#include <gtest/gtest.h>

TEST(FastSafeHashmap, Basic){
    FastSafeHashmap map(2);
    map.update(1, 1);
    map.update(2, 2);

    EXPECT_NO_THROW(ASSERT_EQ(map.get(1).value(), 1));
    EXPECT_NO_THROW(ASSERT_EQ(map.get(2).value(), 2));

    map.update(2, 3);
    EXPECT_NO_THROW(ASSERT_EQ(map.get(2).value(), 3));

    // Collision
    map.update(3, 3);
    EXPECT_NO_THROW(ASSERT_EQ(map.get(3).value(), 3));

    ASSERT_EQ(map.get_size(), 3);

    // Key not found
    ASSERT_EQ(map.get(1000), std::nullopt);
}

TEST(FastSafeHashmap, ParralelInsertion){
    constexpr int n_bucket = 10;
    constexpr int n_thread = 16;
    constexpr int n_data_per_thread = 10;

    FastSafeHashmap map(n_bucket);
    std::vector<std::thread> threads;
    threads.reserve(n_thread);
    for(int i = 0; i < n_thread; ++i){
        threads.emplace_back([&map, i](){
            for (int j = 0; j < n_data_per_thread; ++j) {
                map.update(i * n_data_per_thread + j, i * n_data_per_thread + j);
            }
        });
    }

    for(auto &thread: threads){
        thread.join();
    }

    ASSERT_EQ(map.get_size(), n_thread * n_data_per_thread);

    for(int i = 0; i < n_thread; ++i){
        for(int j = 0; j < n_data_per_thread; ++j){
            ASSERT_EQ(map.get(i * n_data_per_thread + j).value(), i * n_data_per_thread + j);
        }
    }
}

TEST(FastSafeHashmap, ParralelUpdate){
    constexpr int n_bucket = 10;
    constexpr int n_thread = 16;
    constexpr int n_data = 50;

    FastSafeHashmap map(n_bucket);
    std::barrier barrier(n_thread);
    std::vector<std::thread> threads;
    threads.reserve(n_thread);
    for(int i = 0; i < n_thread; ++i){
        threads.emplace_back([&map, &barrier](){
            barrier.arrive_and_wait();
            for (int j = 0; j < n_data; ++j) {
                map.update(j, 1);
            }
        });
    }
    for(auto &thread: threads){
        thread.join();
    }

    // Invalid implementation (update not exclusive) will result in size > n_data
    // because multiple threads will find the absence of key and insert the same data multiple times
    ASSERT_EQ(map.get_size(), n_data);
}

TEST(FastSafeHashmap, ParralelReadWrite){
    constexpr int n_bucket = 10;
    constexpr int n_read_thread = 8;
    constexpr int n_write_thread = 8;

    FastSafeHashmap map(n_bucket);
    for (int i = 0; i < n_read_thread; ++i) {
        map.update(i, i);
        map.update(i + n_read_thread, i + n_read_thread);
    }

    std::barrier barrier(n_read_thread + n_write_thread);
    std::vector<std::thread> read_threads;
    read_threads.reserve(n_read_thread);
    for(int i = 0; i < n_read_thread; ++i){
        read_threads.emplace_back([&map, &barrier](){
            barrier.arrive_and_wait();
            for (int j = 0; j < n_read_thread; ++j) {
                ASSERT_EQ(map.get(j).value(), j);
                ASSERT_EQ(map.get(j + n_read_thread).value(), j + n_read_thread);
            }
        });
    }

    int base = n_read_thread * 2;
    std::vector<std::thread> write_threads;
    write_threads.reserve(n_write_thread);
    for(int i = 0; i < n_write_thread; ++i){
        write_threads.emplace_back([&map, &barrier, base](){
            barrier.arrive_and_wait();
            for (int j = 0; j < n_bucket; ++j) {
                map.update(base + j, 1);
                map.update(base*2 + j, 1);
            }
        });
    }

    for(auto &thread: read_threads){
        thread.join();
    }
    for(auto &thread: write_threads){
        thread.join();
    }
}

template <typename T> requires (std::is_same_v<T, SafeHashmap> || std::is_same_v<T, FastSafeHashmap>)
void readBenchmark(){
    constexpr int n_bucket = 10;
    constexpr int n_thread = 16;
    constexpr int n_data = 5000;

    T map(n_bucket);
    for(int i = 0; i < n_data; ++i){
        map.update(i, i);
    }

    std::vector<std::thread> threads;
    threads.reserve(n_thread);
    for(int i = 0; i < n_thread; ++i){
        threads.emplace_back([&map](){
            for (int j = 0; j < n_data; ++j) {
                map.get(j);
            }
        });
    }

    for(auto &thread: threads){
        thread.join();
    }
}

TEST(FastSafeHashmap, ReadPerformance){
    constexpr int n_repeat = 3;
    constexpr int n_warmup = 3;

    const auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(
        utils::benchmark(n_repeat, n_warmup,readBenchmark<SafeHashmap>)
    );
    GTEST_LOG_(INFO) << "SafeHashmap read performance: " << duration1.count() << "ms";

    const auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(
        utils::benchmark(n_repeat, n_warmup, readBenchmark<FastSafeHashmap>)
    );
    GTEST_LOG_(INFO) << "FastSafeHashmap read performance: " << duration2.count() << "ms";

    ASSERT_LT(duration2.count(), duration1.count());
}

template <typename T> requires (std::is_same_v<T, SafeHashmap> || std::is_same_v<T, FastSafeHashmap>)
void writeBenchmark(){
    constexpr int n_bucket = 10;
    constexpr int n_thread = 16;
    constexpr int n_data = 5000;

    T map(n_bucket);
    std::vector<std::thread> threads;
    threads.reserve(n_thread);
    for(int i = 0; i < n_thread; ++i){
        threads.emplace_back([&map, i](){
            for (int j = 0; j < n_data; ++j) {
                map.update(j, i);
            }
        });
    }

    for(auto &thread: threads){
        thread.join();
    }
}

TEST(FastSafeHashmap, WritePerformance){
    constexpr int n_repeat = 3;
    constexpr int n_warmup = 3;

    const auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(
        utils::benchmark(n_repeat, n_warmup,writeBenchmark<SafeHashmap>)
    );
    GTEST_LOG_(INFO) << "SafeHashmap write performance: " << duration1.count() << "ms";

    const auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(
        utils::benchmark(n_repeat, n_warmup, writeBenchmark<FastSafeHashmap>)
    );
    GTEST_LOG_(INFO) << "FastSafeHashmap write performance: " << duration2.count() << "ms";

    ASSERT_LT(duration2.count(), duration1.count());
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}