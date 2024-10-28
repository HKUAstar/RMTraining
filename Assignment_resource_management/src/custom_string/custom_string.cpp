//
// Created by Cao Rui on 24-9-22.
//
#include <iostream>

#include "custom_string.h"

#include <utils.h>
#include <string.h>

//construct an empty CustomString object
CustomString::CustomString():len(0){
    //No need to assign memory to empty object.
    this->str = nullptr;
}

//construct with a char*
CustomString::CustomString(const char *str_){
    //consider empty str_
    if(str_){
        this->len = strlen(str_);
        // Assign a pointer to another pointer can be dangerous because they point to the same memory
        // And you cannot assign a const pointer to another pointer ... Otherwise you can change the const!
        this->str = new char[this->len + 1];
        strcpy(this->str, str_);
    }
    else{
        char* chars = new char[1];
        chars[0] = '\0';
        this->str = chars;
    }
    
}

//construct with a const CustomString
CustomString::CustomString(const CustomString &str_){
    if(str_.str){
        this->len = str_.len;
        this->str = new char[str_.len + 1];
        strcpy(this->str, str_.str);
    }
    else{
        char* chars = new char[1];
        chars[0] = '\0';
        this->str = chars;
    }
}

//construct with a rvalue of CustomString
CustomString::CustomString(CustomString &&str_) noexcept{
    //recall that && is a rvalue reference
    //Constructor: current instance has not been initialized now!!! You cannot use it directly!!!
    this->str = str_.str;
    this->len = str_.len;
    str_.str = nullptr;
    str_.len = 0;
}

CustomString::~CustomString(){
    if(this->str){
        delete[] this->str;
    }    
    this->len = 0;
}

//a member function of a class has private access to all variables of all instances
void CustomString::swap(CustomString &str_) noexcept{
    using std::swap;
    swap(this->str, str_.str);
    swap(this->len, str_.len);
}

void swap(CustomString &str1, CustomString &str2) noexcept{
    str1.swap(str2);
}

#if (COPY_AND_SWAP)
// The following implemenation follows the copy-and-swap idiom.
CustomString& CustomString::operator=(CustomString str_){
    this->swap(str_);
    return *this;
}
#else
CustomString& CustomString::operator=(const CustomString &str_){
    if(this != &str_){ //Do not forget to include self-assignment
        if(this->str) delete[] this->str;
        this->str = new char[str_.len+1]; //+1 for null-terminator
        strcpy(this->str, str_.str);
        this->len = str_.len;
    }
    return *this;
}
CustomString& CustomString::operator=(CustomString &&str_){
    if(this != &str_){//Do not forget to include self-assignment
        if (this-> str) delete[] this->str;
        this->str = str_.str;
        this->len = str_.len;
        str_.str = nullptr;
        str_.len = 0;
    }
    return *this;
}
#endif

size_t CustomString::length() const{
    //const qualifier force the function not to change any data of the current object.
    //this const has NO relation with the return type of the function
    return this->len;
}

std::string CustomString::to_string() const{
    return std::string (this->str);
}

const char* CustomString::c_str() const{
    return this->str;
}

bool operator ==(const CustomString &str1, const CustomString &str2){
    if(str1.len == str2.len && str1.to_string() == str2.to_string()){
        return true;
    }
    else{
        return false;
    }
}

CustomString operator +(const CustomString &str1, const CustomString &str2){
    /*
    # A typical fault worth reciting: point a pointer to a temporary object, temp_chars, which 
    # will be DELETED after exit this function.
    # How to fix: write the content of the pointer to return DIRECTLY.
    
    CustomString sum = CustomString();
    sum.len = str1.len + str2.len;
    std::string temp_chars = str1.to_string() + str2.to_string();
    sum.str = new char[sum.len + 1]; //ensure enough space for sum. -> Remember to allocate the storage for a newly-created pointer
    sum.str = &temp_chars[0];
    return sum;*/
    CustomString sum;
    sum.len = str1.len + str2.len;
    sum.str = new char[sum.len + 1];
    strcpy(sum.str, str1.str);
    strcat(sum.str, str2.str);
    return sum;
}

CustomString& CustomString::operator +=(const CustomString &str_){
    //remember that all variables created inside this function will be destoryed when return
    //so we need pointers to communicate inside and outside a function returning the reference
    //+= increded after the initial object & modify the current object
    if(this->len){
        this->len += str_.len;
        char* new_str =  new char[this->len + 1];
        strcpy(new_str, this->str);
        strcat(new_str, str_.str);
        delete[] this->str;
        this->str = new_str;
    }
    else{
        //empty current object
        if(this->str) delete[] this->str;
        this->len = str_.len;
        this->str = new char[this->len + 1];
        strcpy(this->str, str_.str);
    }
    return *this;
}

//we want to change the value of current object,
//so we need to return a reference to the exact current object, instead of creating a new object.
char& CustomString::operator [](const size_t index){
    return this->str[index];
}

const char& CustomString::operator [](const size_t index) const{
    return this->str[index]; //?????Unsure, write it by guessing
}