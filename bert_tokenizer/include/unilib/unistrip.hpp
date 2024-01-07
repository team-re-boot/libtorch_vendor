// Copyright 2014 Institute of Formal and Applied Linguistics. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This file is part of UniLib <http://github.com/ufal/unilib/>.
//
// Copyright 2014 Institute of Formal and Applied Linguistics, Faculty of
// Mathematics and Physics, Charles University in Prague, Czech Republic.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// UniLib version: 3.1.2-devel
// Unicode version: 8.0.0

#pragma once

#include <cstdint>

namespace ufal
{
namespace unilib
{

class unistrip
{
public:
  static inline bool is_combining_mark(char32_t chr);
  static inline char32_t strip_combining_marks(char32_t chr);

private:
  static const char32_t CHARS = 0x110000;

  static const uint8_t combining_mark_index[CHARS >> 8];
  static const uint8_t combining_mark_block[][32];

  static const uint8_t stripped_index[CHARS >> 8];
  static const uint16_t stripped_block[][256];
  static const char32_t stripped_data[];
};

bool unistrip::is_combining_mark(char32_t chr)
{
  return chr < CHARS && combining_mark_block[combining_mark_index[chr >> 8]][(chr >> 3) & 0x1F] &
                          (uint8_t(1) << (chr & 0x07));
}

char32_t unistrip::strip_combining_marks(char32_t chr)
{
  if (chr >= CHARS) return chr;
  uint16_t index = stripped_block[stripped_index[chr >> 8]][chr & 0xFF];
  return index ? stripped_data[index] : chr;
}

}  // namespace unilib
}  // namespace ufal
