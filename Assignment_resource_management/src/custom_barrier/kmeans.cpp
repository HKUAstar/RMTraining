//
// Created by Cao Rui on 24-9-4.
//

#include <cassert>
#include <future>
#include <vector>
#include <random>

#include "custom_barrier.h"

static int find_closest_centroid(
    const std::vector<double>& point,
    const std::vector<std::vector<double>>& centroids
){
    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;
    for(int j = 0; j < centroids.size(); j++){
        double dist = 0;
        for(int d = 0; d < point.size(); d++){
            dist += std::pow(point[d] - centroids[j][d], 2);
        }
        if(dist < min_dist){
            min_dist = dist;
            min_idx = j;
        }
    }
    return min_idx;
}

void serial_kmeans(
    const std::vector<std::vector<double>>& data,
    const int k,
    const int total_iter,
    std::vector<std::vector<double>>& centroids
){
    const int num_points = static_cast<int>(data.size());
    const int num_dims = static_cast<int>(data[0].size());
    std::vector<int> counts(k, 0); //create a vector of k elements, each initialized as 0
    std::vector<std::vector<double>> sums(k, std::vector<double>(num_dims, 0));

    // Initialize centroids
    centroids.clear();
    for(int i = 0; i < k; i++){
        centroids.emplace_back(data[i]); //construct an element at the end of a `std::vector`
    }
    for(int iter = 0; iter < total_iter; iter++) {
        // Assign
        for(int i = 0; i < num_points; i++){
            const int min_idx = find_closest_centroid(data[i], centroids);
            counts[min_idx]++;
            for(int d = 0; d < num_dims; d++){
                sums[min_idx][d] += data[i][d];
            }
        }

        // Update centroids
        for(int i = 0; i < k; i++){
            for(int d = 0; d < num_dims; d++){
                centroids[i][d] = sums[i][d] / counts[i];

                // Clear sum for next iteration
                sums[i][d] = 0;
            }
            // Clear count for next iteration
            counts[i] = 0;
        }
    }
}


static void parallel_kmeans_task(
    const std::vector<std::vector<double>>& data,
    const int k,
    const int total_iter,
    std::vector<std::vector<double>>& centroids,
    std::vector<int> &label,
    const int thread_id,
    const int world_size,
    CustomBarrier& barrier
){
    const int num_points = static_cast<int>(data.size());
    const int num_dims = static_cast<int>(data[0].size());
    const int start = thread_id * num_points / world_size;
    const int end = (thread_id + 1) * num_points / world_size;
    const int start_centroid = thread_id * k / world_size;
    const int end_centroid = (thread_id + 1) * k / world_size;

    for(int iter = 0; iter < total_iter; iter++) {
        for(int i = start; i < end; i++) {
            label[i] = find_closest_centroid(data[i], centroids);
        }
        barrier.wait();

        for(int i = start_centroid; i < end_centroid; i++) {
            std::vector<double> sum(num_dims, 0);
            int count = 0;
            for(int j = 0; j < num_points; j++) {
                if(label[j] == i) {
                    count++;
                    for(int d = 0; d < data[0].size(); d++) {
                        sum[d] += data[j][d];
                    }
                }
            }
            for(int d = 0; d < num_dims; d++) {
                centroids[i][d] = sum[d] / count;
            }
        }

        barrier.wait();
    }
}

void parallel_kmeans(
    const std::vector<std::vector<double>>& data,
    const int k,
    const int total_iter,
    std::vector<std::vector<double>>& centroids,
    const int world_size
){
    assert(k % world_size == 0);
    assert(data.size() % world_size == 0);

    // Initialize centroids
    centroids.clear();
    for(int i = 0; i < k; i++){
        centroids.emplace_back(data[i]);
    }

    CustomBarrier barrier(world_size);
    std::vector<int> label(data.size(), 0);

    std::vector<std::thread> futures;
    futures.reserve(world_size);
    for(int i = 0; i < world_size; i++) {
        futures.emplace_back(
            parallel_kmeans_task,
            std::ref(data),
            k, total_iter,
            std::ref(centroids),
            std::ref(label),
            i,
            world_size,
            std::ref(barrier)
        );
    }
    for(auto& future: futures) {
        future.join();
    }
}

