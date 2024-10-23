//
// Created by Cao Rui on 24-9-22.
//
#include <iostream>

#include <cstring>

#include "custom_string.h"

#include <utils.h>

CustomString::CustomString():len(0){
    str = nullptr;
    len = 0;
}

CustomString::CustomString(const char *str_){
	len = strlen(str_);
    str = new char[len + 1];
    strcpy(str, str_);
}

CustomString::CustomString(const CustomString &str_){
	len = str_.len;
    str = new char[len + 1];
    strcpy(str, str_.str);
}

CustomString::CustomString(CustomString &&str_) noexcept{
    str = str_.str;
    len = str_.len;
    str_.str = nullptr;
    str_.len = 0;
}

CustomString::~CustomString(){
    delete[] str;
    len = 0;
}

void CustomString::swap(CustomString &str_) noexcept{
    std::swap(str, str_.str);
    std::swap(len, str_.len);
}

void swap(CustomString &str1, CustomString &str2) noexcept{
    str1.swap(str2);
}

#if (COPY_AND_SWAP)
// The following implemenation follows the copy-and-swap idiom.
CustomString& CustomString::operator=(CustomString str_){
    swap(str_);
    return *this;
}
#else
CustomString& CustomString::operator=(const CustomString &str_){
    if (this == &str_) {
        return *this;
    }
    delete[] str;
    len = str_.len;
    str = new char[len + 1];
    strcpy(str, str_.str);
    return *this;
}
CustomString& CustomString::operator=(CustomString &&str_){
    if (this == &str_) {
        return *this;
    }
    delete[] str;
    str = str_.str;
    len = str_.len;
    str_.str = nullptr;
    str_.len = 0;
    return *this;
}
#endif

size_t CustomString::length() const{
    return len;
}

std::string CustomString::to_string() const{
    return std::string(str);
}

const char* CustomString::c_str() const{
    return str;
}

bool operator ==(const CustomString &str1, const CustomString &str2){
    if (str1.len != str2.len) {
        return false;
    }
    return strcmp(str1.str, str2.str) == 0;
}

CustomString operator +(const CustomString &str1, const CustomString &str2){
    size_t new_len = str1.len + str2.len;
    char* new_str = new char[new_len + 1];
    strcpy(new_str, str1.str);
    strcat(new_str, str2.str);
    CustomString result(new_str);
    delete[] new_str;
    return result;
}

CustomString& CustomString::operator +=(const CustomString &str_){
    size_t new_len = len + str_.len;
    char* new_str = new char[new_len + 1];
    strcpy(new_str, str);
    strcat(new_str, str_.str);
    delete[] str;
    str = new_str;
    len = new_len;
    return *this;
}

char& CustomString::operator [](const size_t index){
    if (index >= len) {
        throw std::out_of_range("Index out of range");
    }
    return str[index];
}

const char& CustomString::operator [](const size_t index) const{
    if (index >= len) {
        throw std::out_of_range("Index out of range");
    }
    return str[index];
}