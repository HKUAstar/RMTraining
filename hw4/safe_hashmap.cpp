//
// Created by Cao Rui on 24-9-8.
//

#include "safe_hashmap.h"

#include <utils.h>
#include <mutex>
#include <thread>
#include <condition_variable>

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
//speed up by giving one lock per bucket

FastSafeHashmap::FastSafeHashmap(const size_t n_bucket):table(n_bucket, nullptr), access_lock_table(n_bucket), n_bucket(n_bucket), size(0){}

FastSafeHashmap::~FastSafeHashmap(){
    for (auto const &bucket: table) {
        const entry *cur = bucket;
        while (cur) {
            const entry *next = cur->next;
            delete cur;
            cur = next;
        }
    }
}

void FastSafeHashmap::update(const int key, const int value){
    const size_t bucket_id = key % n_bucket;

    //entry condition
    std::unique_lock<std::mutex> lck (access_lock_table[bucket_id]);

    entry *cur = table[bucket_id];
    while (cur != nullptr) {
        if (cur->key == key) {
            cur->value = value;
            return;
        }
        cur = cur->next;
    }
    auto *new_entry = new entry{key, value, nullptr};
    new_entry->next = table[bucket_id];
    table[bucket_id] = new_entry;
    ++size;
}

std::optional<int> FastSafeHashmap::get(const int key){
    const size_t bucket_id = key % n_bucket;

    //enter condition
    std::unique_lock<std::mutex> lck(access_lock_table[bucket_id]);

    //read operation
    const entry *cur = table[bucket_id];
    while (cur != nullptr) {
        if (cur->key == key) {
            return {cur->value};
        }
        cur = cur->next;
    }
    //exit conditions
    return std::nullopt;
}

size_t FastSafeHashmap::get_size() const{
    return size;
}