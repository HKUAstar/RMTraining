//
// Created by Cao Rui on 24-9-5.
//
#include <gtest/gtest.h>
#include "custom_string.h"
#include "utils.h"

TEST(CustomString, Basic){
    CustomString str1("test"), str2("test"), str3("Test");
    EXPECT_EQ(str1.length(), 4);
    EXPECT_EQ(str1.to_string(), "test");
    EXPECT_EQ(str1, str1);
    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1, str3);
    str3[0] = 't';
    EXPECT_EQ(str3.to_string(), "test");
    EXPECT_EQ(str1, str3);
}

TEST(CustomString, NoMemorySharing){
    char p[] = "test";
    CustomString str1(p), str2(p);
    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());

    p[0] = 'T';
    EXPECT_EQ(str1.to_string(), "test");
    str1[0] = 'T';
    EXPECT_EQ(str1.to_string(), "Test");
    EXPECT_EQ(str2.to_string(), "test");
}

TEST(CustomString, CopyAndMove){

    const CustomString str1("test");

    // Copy Test
    EXPECT_TRUE(std::is_copy_constructible_v<CustomString>);
    CustomString str2(str1);
    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
    str2[0] = 'T';
    EXPECT_NE(str1, str2);

    // Move Correctness Test
    EXPECT_TRUE(std::is_move_constructible_v<CustomString>);
    const char *p = str2.c_str();
    CustomString str3(std::move(str2));
    EXPECT_EQ(str3.to_string(), "Test");
    EXPECT_EQ(str3.c_str(), p);

    // Move Performance Test
    constexpr int len = 10000000;
    const std::string std_str(len, 'a');
    CustomString str(std_str.c_str());
    auto copy_test = [](CustomString str){
        CustomString str2(str);
        return str2;
    };
    auto move_test = [](CustomString str){
        CustomString str2(std::move(str));
        return str2;
    };

    auto copy_time = utils::benchmark(5, 3, copy_test, str);
    auto move_time = utils::benchmark(5, 3, move_test, str);
    GTEST_LOG_(INFO) << "Time copy: "
    << std::chrono::duration_cast<std::chrono::microseconds>(copy_time).count() << "[us]";
    GTEST_LOG_(INFO) << "Time move: "
        << std::chrono::duration_cast<std::chrono::microseconds>(move_time).count() << "[us]";
    GTEST_LOG_(INFO) << "Speedup: " << copy_time.count() / move_time.count() << "x";

    EXPECT_LT(move_time, copy_time);
}

TEST(CustomString, Assignment){
    EXPECT_TRUE(std::is_copy_assignable_v<CustomString>);
    EXPECT_TRUE(std::is_move_assignable_v<CustomString>);

    // Copy Assignment
    CustomString str1("test"), str2("Test");
    str1 = str2;
    EXPECT_EQ(str1, str2);
    EXPECT_NE(str1.c_str(), str2.c_str());
    str1 = str1;    // Self-assignment test, incorrect implementation will cause memory leak or content loss.
    EXPECT_EQ(str1.to_string(), "Test");

    // Move Assignment
    CustomString str3("test"), str4("Test");
    const char *p = str4.c_str();
    str3 = std::move(str4);
    EXPECT_EQ(str3.to_string(), "Test");
    EXPECT_EQ(str3.c_str(), p);
    str3 = std::move(str3);    // Self-assignment test
    EXPECT_EQ(str3.to_string(), "Test");
}

TEST(CustomString, Concat){
    CustomString str1("test"), str2("Test");
    CustomString str3 = str1 + str2;
    EXPECT_EQ(str3.to_string(), "testTest");
    str1 += str2;
    EXPECT_EQ(str1.to_string(), "testTest");
}


int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);


    return RUN_ALL_TESTS();
}