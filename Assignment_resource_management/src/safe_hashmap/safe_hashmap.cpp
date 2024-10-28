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

FastSafeHashmap::FastSafeHashmap(const size_t n_bucket): table(n_bucket, nullptr), n_bucket(n_bucket), size(0), entry_lock(n_bucket){}

FastSafeHashmap::~FastSafeHashmap(){
    for(entry* bucket: table){
        entry* cur = bucket;
        while(cur){
            entry*next = cur->next;
            delete cur;
            cur = next;
        }
    }
    size = 0;
}

//update hash[k_value] to value
void FastSafeHashmap::update(const int key, const int value){
    const size_t bucket_id = key % n_bucket; //the hash function is k % n_bucket

    std::unique_lock<std::mutex> lock(entry_lock[bucket_id]);

    entry* cur = table[bucket_id];
    bool not_found = true;
    while(cur != nullptr){
        if(cur->key == key){
            cur->value = value;
            return;
        }
        cur = cur->next;
    }

    //if the key is not found, insert a new one
    entry* en= new entry{key, value, nullptr};
    en->next = table[bucket_id];
    table[bucket_id] = en;
    //increment the size
    size++;    
}

std::optional<int> FastSafeHashmap::get(const int key){
    const size_t bucket_id = key % n_bucket;
    std::unique_lock<std::mutex> lock(entry_lock[bucket_id]);

    entry* cur = table[bucket_id];
    while(cur != nullptr){
        if(cur->key == key){
            return cur->value;
        }
        cur = cur->next;
    }
    return std::nullopt;
}

size_t FastSafeHashmap::get_size() const{
    return size;
}


//Reference of the granularity of lock: https://www.geeksforgeeks.org/multiple-granularity-locking-in-dbms/