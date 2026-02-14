#pragma once

#include <unordered_map>

template <typename T> using consts_t = std::unordered_map<std::string, T>;

template <typename T = double>
concept NumericType = std::integral<T> || std::floating_point<T>;
