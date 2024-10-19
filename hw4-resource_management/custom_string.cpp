//
// Created by Cao Rui on 24-9-22.
//
#include <iostream>

#include "custom_string.h"

#include <utils.h>

#include <string.h>

CustomString::CustomString():len(0){
    str = nullptr;
}

//default constructor
CustomString::CustomString(const char *str_){
    str = new char[strlen(str_)+MAX_STRING_LEN];
    strcpy(str, str_);
    len = strlen(str_);
}

//copy constructor
CustomString::CustomString(const CustomString &str_){
    str = new char[str_.len+MAX_STRING_LEN];
    len = str_.len;
    strcpy(str, str_.str);
}

//move constructor
CustomString::CustomString(CustomString &&str_) noexcept{
    str = str_.str;
    len = str_.len;
    str_.str = nullptr;
    str_.len = 0;
}

//destructor
CustomString::~CustomString(){
    delete[] str;
    len = 0;
}

void CustomString::swap(CustomString &str_) noexcept{
    // char *temp = str;
    // int tempLen = len;
    // str = str_.str;
    // len = str_.len;
    // str_.str = temp;
    // str_.len = tempLen;
    std::swap(str, str_.str);
    std::swap(len, str_.len);
}

void swap(CustomString &str1, CustomString &str2) noexcept{
    // char *temp = str1.str;
    // int tempLen = str1.len;
    // str1.str = str2.str;
    // str1.len = str2.len;
    // str2.str = temp;
    // str2.len = tempLen;
    // std::swap(str1.str, str2.str);
    // std::swap(str1.len, str2.len);
    str1.swap(str2);
}

#if (COPY_AND_SWAP)
// The following implemenation follows the copy-and-swap idiom.
// Answer to extra question: copy-and-swap is more efficient,
// because it simply exchange the values of two pointers, this->str and str_.str.
// No extra memory allocation and deletion is required.

CustomString& CustomString::operator=(CustomString str_){
    swap(str_);
    return *this;
}
#else
//lvalue
CustomString& CustomString::operator=(const CustomString &str_){
    if (this != &str_) { //prevent self-assignment
        delete[] str;

        str = new char[str_.len+MAX_STRING_LEN];

        strcpy(str, str_.str);
        len = str_.len;
    } 
    return *this;
}
//rvalue
CustomString& CustomString::operator=(CustomString &&str_){
    if (this != &str_) { //prevent self-assignment
        if (str)
            delete[] str;

        str = str_.str;
        len = str_.len;

        str_.str = nullptr;
        str_.len = 0;
    }
    return *this;
}
#endif

size_t CustomString::length() const{
    return len;
}

std::string CustomString::to_string() const{
    std::string my_string(str);
    return my_string;
}

const char* CustomString::c_str() const{
    return str;
}

bool operator ==(const CustomString &str1, const CustomString &str2){
    return strcmp(str1.str, str2.str) == 0;
}

CustomString operator +(const CustomString &str1, const CustomString &str2){
    char *temp = new char[str1.len+str2.len+MAX_STRING_LEN];
    strcpy(temp, str1.str);
    strcat(temp, str2.str);
    CustomString ret = CustomString(temp);
    delete[] temp;
    return ret;
}

CustomString& CustomString::operator +=(const CustomString &str_){
    char *temp = new char[len + str_.len+MAX_STRING_LEN];
    strcpy(temp, str);
    strcat(temp, str_.str);
    delete[] str;
    str = temp;
    return *this;
}

//modify
char& CustomString::operator [](const size_t index){
    if (index > len) {
        throw std::out_of_range("Index out of bounds");
    }
     return str[index];
}

//read
const char& CustomString::operator [](const size_t index) const{
    if (index > len) {
        throw std::out_of_range("Index out of bounds");
    }
    return str[index];
}