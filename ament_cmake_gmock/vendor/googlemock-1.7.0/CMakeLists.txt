project(gmock CXX C)
cmake_minimum_required(VERSION 2.6.2)

include_directories(include)

add_library(gmock src/gmock-gtest-all.cc)

add_library(gmock_main src/gmock-gtest-all.cc src/gmock_main.cc)
