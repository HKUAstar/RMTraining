//
// Created by Cao Rui on 24-8-30.
//
#include <future>
#include <iostream>
#include <random>
#include <gtest/gtest.h>
#include <chrono>

#include "utils.h"
#include "custom_barrier.h"

TEST(CustomBarrier, Basic){
    CustomBarrier barrier(2);
    std::atomic<int> flag{0};
    auto future1 = std::async([&](){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        barrier.wait();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        flag.store(1);
        return flag.load();
    });

    auto future2 = std::async([&](){
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        barrier.wait();
        return flag.load();
    });
    EXPECT_EQ(future1.get(), 1);
    EXPECT_EQ(future2.get(), 0);
}

TEST(CustomBarrier, IsCopyable){
    EXPECT_FALSE(std::is_copy_constructible_v<CustomBarrier>);
    EXPECT_FALSE(std::is_copy_assignable_v<CustomBarrier>);
}

extern void serial_kmeans(
    const std::vector<std::vector<double>>& data,
    const int k,
    const int total_iter,
    std::vector<std::vector<double>>& centroids
);

extern void parallel_kmeans(
    const std::vector<std::vector<double>>& data,
    const int k,
    const int total_iter,
    std::vector<std::vector<double>>& centroids,
    const int world_size
);

TEST(CustomBarrier, Kmeans){
    constexpr int world_size = 32;
    constexpr int k = 32;
    constexpr int dims = 3;
    constexpr int num_points = world_size * 500;
    constexpr int total_iter = 20;

    /************* Data Generation ***********/
    // Normal distribution random number generator
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<double> distribution(0.0, 1.0);

    std::vector<std::vector<double>> centroid_gt;
    std::vector<std::vector<double>> data;
    // Generate ground truth centroids
    for (int i = 0; i < k; i++) {
        std::vector<double> centroid(dims, 0.0);
        centroid[0] = static_cast<double>(i);
        centroid_gt.emplace_back(centroid);
    }
    // Generate data points
    for (int i = 0; i < num_points; i++) {
        const int centroid_id = i % k;
        std::vector<double> point(dims, 0.0);
        for (int d = 0; d < dims; d++) {
            point[d] = distribution(generator) + centroid_gt[centroid_id][d];
        }
        data.emplace_back(point);
    }


    /**************** Run Kmeans ***************/
    // Serialized Kmeans
    std::vector<std::vector<double>> centroid_serial;
    serial_kmeans(data, k, total_iter, centroid_serial);

    // Parallel Kmeans (Barrier is tested here as a key component of parallel kmeans)
    std::vector<std::vector<double>> centroid_parallel;
    parallel_kmeans(data, k, total_iter, centroid_parallel, world_size);


    /************** Check Results ************/
    for (int i = 0; i < k; i++) {
        for (int d = 0; d < dims; d++) {
            EXPECT_NEAR(centroid_serial[i][d], centroid_parallel[i][d], 1e-6);
        }
    }

    /************** Performance Benchmark ************/
    constexpr int n_repeat = 5;
    constexpr int n_warmup = 3;
    const auto time_serial = utils::benchmark(
        n_repeat,
        n_warmup,
        serial_kmeans,
        data,
        k,
        total_iter,
        centroid_serial
    );
    const auto time_parallel = utils::benchmark(
        n_repeat,
        n_warmup,
        parallel_kmeans,
        data,
        k,
        total_iter,
        centroid_parallel,
        world_size
    );

    GTEST_LOG_(INFO) << "Time serial: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(time_serial).count() << "[ms]";
    GTEST_LOG_(INFO) << "Time parallel: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(time_parallel).count() << "[ms]";
    GTEST_LOG_(INFO) << "Speedup: " << time_serial.count() / time_parallel.count() << "x";

    EXPECT_LT(time_parallel.count(), time_serial.count());
}

int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}