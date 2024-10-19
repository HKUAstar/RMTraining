//
// Created by Cao Rui on 24-8-22.
//

#ifndef CUSTOM_STRING_H
#define CUSTOM_STRING_H

#define COPY_AND_SWAP 0
#define MAX_STRING_LEN 64

/**
 * @brief A custom string class wrapper for char * string.
 * @details Design principle: Every object will have its own exclusive memory space for char *. No memory sharing between different objects and with other char* / std::string.
 */
class CustomString{
private:
    char *str;
    size_t len;

public:
    CustomString();

    explicit CustomString(const char *str_);

    CustomString(const CustomString &str_);

    CustomString(CustomString &&str_) noexcept;

    /**
     * @brief Copy & Move assignment operator.
     * @details Follow the copy-and-swap idiom.
     * @param str_ The CustomString object to copy/move from.
     * @return The reference to the current object.
     */
#if (COPY_AND_SWAP)
    CustomString& operator=(CustomString str_);
#else
    CustomString& operator=(const CustomString &str_);
    CustomString& operator=(CustomString &&str_);
#endif

    virtual ~CustomString();

    /**
     * @brief Swap the content of the current object with another CustomString object.
     * @param str_ The CustomString object to swap with.
     */
    void swap(CustomString &str_) noexcept;

    [[nodiscard]] size_t length() const;
    [[nodiscard]] std::string to_string() const;
    [[nodiscard]] const char* c_str() const;

    friend bool operator ==(const CustomString &str1, const CustomString &str2);
    friend CustomString operator +(const CustomString &str1, const CustomString &str2);
    CustomString& operator +=(const CustomString &str_);
    char& operator [](const size_t index);
    const char& operator [](const size_t index) const;
};

void swap(CustomString &str1, CustomString &str2) noexcept;


#endif //CUSTOM_STRING_H
