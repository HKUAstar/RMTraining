//
// Created by Cao Rui on 24-9-8.
//

#include "safe_hashmap.h"
#include <utils.h>

/**************** SafeHashmap Sample ****************/

SafeHashmap::SafeHashmap(size_t n_bucket) : table(n_bucket, nullptr), n_bucket(n_bucket), size(0) {}

SafeHashmap::~SafeHashmap() {
    for (auto const& bucket : table) {
        entry* cur = bucket;
        while (cur) {
            entry* next = cur->next;
            delete cur;
            cur = next;
        }
    }
}

void SafeHashmap::update(int key, int value) {
    const size_t bucket_id = key % n_bucket;
    std::unique_lock<std::mutex> lock(global_lock); // 获取全局锁

    entry* cur = table[bucket_id];
    while (cur != nullptr) {
        if (cur->key == key) {
            cur->value = value;
            return;
        }
        cur = cur->next;
    }

    // 如果key不存在，插入新entry
    entry* new_entry = new entry{key, value, table[bucket_id]};
    table[bucket_id] = new_entry;
    size++;
}

std::optional<int> SafeHashmap::get(int key) {
    const size_t bucket_id = key % n_bucket;
    std::lock_guard<std::mutex> lock(global_lock); // 获取全局锁

    entry* cur = table[bucket_id];
    while (cur != nullptr) {
        if (cur->key == key) {
            return cur->value;
        }
        cur = cur->next;
    }
    return std::nullopt;
}

size_t SafeHashmap::get_size() const {
    return size;
}

/**************** FastSafeHashmap ****************/

FastSafeHashmap::FastSafeHashmap(size_t n_bucket) : table(n_bucket, nullptr), n_bucket(n_bucket), size(0), bucket_locks(n_bucket) {}

FastSafeHashmap::~FastSafeHashmap() {
    for (auto const& bucket : table) {
        entry* cur = bucket;
        while (cur) {
            entry* next = cur->next;
            delete cur;
            cur = next;
        }
    }
}

void FastSafeHashmap::update(int key, int value) {
    const size_t bucket_id = key % n_bucket;
    std::unique_lock<std::shared_mutex> lock(bucket_locks[bucket_id]); // 获取分段写锁

    entry* cur = table[bucket_id];
    while (cur != nullptr) {
        if (cur->key == key) {
            cur->value = value;
            return;
        }
        cur = cur->next;
    }

    // 如果key不存在，插入新entry
    entry* new_entry = new entry{key, value, table[bucket_id]};
    table[bucket_id] = new_entry;
    size++;
}

std::optional<int> FastSafeHashmap::get(int key) {
    const size_t bucket_id = key % n_bucket;
    std::shared_lock<std::shared_mutex> lock(bucket_locks[bucket_id]); // 获取分段读锁

    entry* cur = table[bucket_id];
    while (cur != nullptr) {
        if (cur->key == key) {
            return cur->value;
        }
        cur = cur->next;
    }
    return std::nullopt;
}

size_t FastSafeHashmap::get_size() const {
    return size;
}