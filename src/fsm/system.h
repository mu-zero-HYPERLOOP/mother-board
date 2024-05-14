#pragma once

#include "canzero/canzero.h"
#include "range.h"
#include <algorithm>
#include <array>

#include "canzero.h"

template <typename T, size_t N, std::array<T (*)(), N> prodivers>
struct System {
  inline static bool all(T state) {
    return std::all_of(prodivers.begin(), prodivers.end(),
                       [&](auto provider) { return provider() == state; });
  }

  inline static bool any(T state) {
    return std::any_of(prodivers.begin(), prodivers.end(),
                       [&](auto provider) { return provider() == state; });
  }

  inline static bool any_not(T state) {
    return !all(state);
  }

  inline static bool none(T state) { return !any(state); }

  template <size_t M> inline static bool all_in(const Range<T, M> &range) {
    return std::all_of(prodivers.begin(), prodivers.end(), [&](auto provider) {
      return range.contains(provider());
    });
  }

  template <size_t M> inline static bool any_in(const Range<T, M> &range) {
    return std::any_of(prodivers.begin(), prodivers.end(), [&](auto provider) {
      return range.contains(provider());
    });
  }

  template <size_t M> inline static bool any_not_in(const Range<T, M> &range) {
    return !all_in(range);
  }

  template <size_t M> inline static bool none_in(const Range<T, M> &range) {
    return !any_in(range);
  }
};
