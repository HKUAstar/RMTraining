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
    const int NumOfLocks = 37;
    struct entry{
        int key, value;
        entry *next;
    };
    std::vector<entry*> table[37];
    std::mutex locks[37];

    const size_t n_bucket;
    size_t size;
    std::mutex global_lock;

    size_t hash(int key) const {
        return key % NumOfLocks;
    }

public:
    explicit FastSafeHashmap(size_t n_bucket);
    ~FastSafeHashmap();

    void update(int key, int value);
    std::optional<int> get(int key);
    [[nodiscard]] size_t get_size() const;
};

#endif //SAFE_HASHMAP_H
