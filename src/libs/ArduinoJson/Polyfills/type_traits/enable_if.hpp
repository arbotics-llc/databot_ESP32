// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2022, Benoit BLANCHON
// MIT License

#pragma once

// #include <ArduinoJson/Namespace.hpp>
// #include "../Namespace.hpp"
 #include "libs/ArduinoJson/Namespace.hpp"

namespace ARDUINOJSON_NAMESPACE {

// A meta-function that return the type T if Condition is true.
template <bool Condition, typename T = void>
struct enable_if {};

template <typename T>
struct enable_if<true, T> {
  typedef T type;
};
}  // namespace ARDUINOJSON_NAMESPACE
