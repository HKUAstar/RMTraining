//
// Created by Cao Rui on 24-9-22.
//
#include <iostream>

#include "custom_string.h"

#include <utils.h>
// 1.
CustomString::CustomString(){
    len = 0;
    str = new char[1];
    str[0] = '\0';
}
// 2.
CustomString::CustomString(const char *str_){
    len = strlen(str_);
    str = new char[len + 1];
    // std::cout<<"CustomString(const char *str_): "<<str_<<std::endl;
    // std::cout<<"len: "<<len<<std::endl;
    std::strcpy(str, str_);
}
// 4.
CustomString::CustomString(const CustomString &str_){
    len = str_.len;
    str = new char[len + 1];
    if (str_.str) {
        std::strcpy(str, str_.str);
    } else {
        str[0] = '\0';
    }
}
// 5.
CustomString::CustomString(CustomString &&str_) noexcept{
    str = str_.str;
    len = str_.len;
    str_.str = nullptr;
    str_.len = 0;
}
// 3.
CustomString::~CustomString(){
    delete[] str;
}
// 6.
void CustomString::swap(CustomString &str_) noexcept{
    std::swap(str, str_.str);
    std::swap(len, str_.len);
}
// 6.
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
    if (this != &str_) {
        char *new_str = new char[str_.len + 1];
        std::strcpy(new_str, str_.str);
        delete[] str;
        str = new_str;
        len = str_.len;
    }
    return *this;
}
CustomString& CustomString::operator=(CustomString &&str_){
    if (this != &str_) {
        delete[] str;
        str = str_.str;
        len = str_.len;
        str_.str = nullptr;
        str_.len = 0;
    }
    return *this;
}
#endif
// 7.
size_t CustomString::length() const{
    return len;
}    

std::string CustomString::to_string() const{
    return std::string(str);
}

const char* CustomString::c_str() const{
    return str;
}
// 9.
bool operator ==(const CustomString &str1, const CustomString &str2){
    return std::strcmp(str1.str ? str1.str : "", str2.str ? str2.str : "") == 0;
}
// 8.
CustomString operator+(const CustomString &str1, const CustomString &str2) {
    size_t new_len = str1.len + str2.len;
    char *new_str = new char[new_len + 1];
    std::strcpy(new_str, str1.str ? str1.str : "");
    std::strcat(new_str, str2.str ? str2.str : "");
    CustomString result(new_str);
    delete[] new_str;
    return result;
}

CustomString& CustomString::operator +=(const CustomString &str_){
    *this = *this + str_;
    return *this;
}
// 10.
char& CustomString::operator[](size_t index) {
    return str[index];
}

const char& CustomString::operator[](size_t index) const {
    return str[index];
}