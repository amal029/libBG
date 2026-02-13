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
using consts_t = const std::unordered_map<std::string_view, T, StringViewHash,
                                          StringViewEqual>;
