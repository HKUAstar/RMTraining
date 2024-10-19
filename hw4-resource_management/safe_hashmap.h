//
// Created by Cao Rui on 24-9-8.
//

#ifndef SAFE_HASHMAP_H
#define SAFE_HASHMAP_H
#include <atomic>
#include <vector>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <condition_variable>
#include <semaphore.h>

class SafeHashmap{
private:
    struct entry{
        int key, value;
        entry *next;
    };

    std::vector<entry*> table;
    const size_t n_bucket;
    size_t size;
    std::mutex global_lock;

public:
    explicit SafeHashmap(size_t n_bucket);
    ~SafeHashmap();

    void update(int key, int value);
    std::optional<int> get(int key);
    [[nodiscard]] size_t get_size() const;
};

class FastSafeHashmap{
private:
    struct entry{
        int key, value;
        entry *next;
    };
    std::vector<entry*> table;
    const size_t n_bucket;
    size_t size;
    std::vector<std::mutex> access_lock_table;
    
    

public:
    explicit FastSafeHashmap(size_t n_bucket);
    ~FastSafeHashmap();

    void update(int key, int value);
    std::optional<int> get(int key);
    [[nodiscard]] size_t get_size() const;
};

#endif //SAFE_HASHMAP_H
