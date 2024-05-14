#pragma once

#include <array>
#include <cstddef>

template <typename T, size_t N> struct Range {
public:
  template <typename... E,
            typename = typename std::enable_if_t<
                std::conjunction_v<std::is_convertible<E, T>...> &&
                    N == (1 + sizeof...(E)),
                void>>
  __attribute__((always_inline)) constexpr Range(T v, E &&...elements) noexcept
      : m_range({{v, static_cast<T>(std::forward<E>(elements))...}}) {}

  bool contains(T value) {
    for (size_t i = 0; i < N; i += 1) {
      if (m_range[i] == value) {
        return true;
      }
    }
    return false;
  }

private:
  std::array<T, N> m_range;
};
