// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2022, Benoit BLANCHON
// MIT License

#pragma once

// #include <ArduinoJson/StringStorage/StringCopier.hpp>
// #include <ArduinoJson/StringStorage/StringMover.hpp>
#include "../StringStorage/StringCopier.hpp"
#include "../StringStorage/StringMover.hpp"
namespace ARDUINOJSON_NAMESPACE {

template <typename TInput>
StringCopier makeStringStorage(TInput&, MemoryPool& pool) {
  return StringCopier(pool);
}

template <typename TChar>
StringMover makeStringStorage(
    TChar* input, MemoryPool&,
    typename enable_if<!is_const<TChar>::value>::type* = 0) {
  return StringMover(reinterpret_cast<char*>(input));
}
}  // namespace ARDUINOJSON_NAMESPACE
