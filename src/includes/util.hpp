#pragma once

#include <string_view>
#include <unordered_map>

struct StringViewHash {
  std::size_t operator()(std::string_view str) const noexcept {
    return std::hash<std::string_view>{}(str);
  }
};

struct StringViewEqual {
  bool operator()(std::string_view lhs, std::string_view rhs) const noexcept {
    return lhs == rhs;
  }
};

template <typename T>
using consts_t =
    std::unordered_map<std::string_view, T, StringViewHash, StringViewEqual>;

template <typename T = double>
concept NumericType = std::integral<T> || std::floating_point<T>;
