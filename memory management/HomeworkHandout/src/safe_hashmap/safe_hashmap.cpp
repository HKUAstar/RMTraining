//
// Created by Cao Rui on 24-9-8.
//

#include "safe_hashmap.h"

#include <utils.h>

/**************** SafeHashmap Sample ****************/

SafeHashmap::SafeHashmap(const size_t n_bucket): table(n_bucket, nullptr), n_bucket(n_bucket), size(0){}

SafeHashmap::~SafeHashmap(){
    for(auto const &bucket: table){
        const entry *cur = bucket;
        while(cur){
            const entry *next = cur->next;
            delete cur;
            cur = next;
        }
    }
}

void SafeHashmap::update(const int key, const int value){
    const size_t bucket_id = key % n_bucket;
    std::lock_guard<std::mutex> lock(global_lock);          // Acquire global lock

    entry *cur = table[bucket_id];
    while(cur != nullptr){
        if(cur->key == key){
            cur->value = value;
            return;
        }
        cur = cur->next;
    }

    // If key not found, insert a new entry
    auto *new_entry = new entry{key, value, nullptr};
    new_entry->next = table[bucket_id];
    table[bucket_id] = new_entry;
    ++size;
}

std::optional<int> SafeHashmap::get(const int key){
    const size_t bucket_id = key % n_bucket;
    std::lock_guard<std::mutex> lock(global_lock);          // Acquire global lock

    const entry *cur = table[bucket_id];
    while(cur != nullptr){
        if(cur->key == key){
            return {cur->value};
        }
        cur = cur->next;
    }
    return std::nullopt;
}

size_t SafeHashmap::get_size() const{
    return size;
}


/**************** FastSafeHashmap ****************/

FastSafeHashmap::FastSafeHashmap(const size_t n_bucket) : n_bucket(n_bucket), size(0) {
    for (int i = 0; i < NumOfLocks; ++i) {
        table[i] = std::vector<entry*>();
    }
}

FastSafeHashmap::~FastSafeHashmap(){
    for (int i = 0; i < NumOfLocks; ++i) {
        for(auto const &bucket: table[i]){
            const entry *cur = bucket;
            while(cur){
                const entry *next = cur->next;
                delete cur;
                cur = next;
            }
        }
    }
}

void FastSafeHashmap::update(const int key, const int value){
    size_t index = hash(key);
    std::lock_guard<std::mutex> lock(locks[index]);

    entry *head = table[index].empty() ? nullptr : table[index][0];
    while (head) {
        if (head->key == key) {
            head->value = value;
            return;
        }
        head = head->next;
    }

    entry *new_entry = new entry{key, value, table[index].empty() ? nullptr : table[index][0]};
    if (table[index].empty()) {
        table[index].push_back(new_entry);
    } else {
        table[index][0] = new_entry;
    }
    ++size;
}

std::optional<int> FastSafeHashmap::get(const int key){
    size_t index = hash(key);
    std::lock_guard<std::mutex> lock(locks[index]);

    entry *head = table[index].empty() ? nullptr : table[index][0];
    while (head) {
        if (head->key == key) {
            return head->value;
        }
        head = head->next;
    }
    return std::nullopt;
}

size_t FastSafeHashmap::get_size() const{
    return size;
}