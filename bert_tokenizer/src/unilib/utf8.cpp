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

#include "utf8.hpp"

namespace ufal
{
namespace unilib
{

bool utf8::valid(const char * str)
{
  for (; *str; str++)
    if (((unsigned char)*str) >= 0x80) {
      if (((unsigned char)*str) < 0xC0)
        return false;
      else if (((unsigned char)*str) < 0xE0) {
        str++;
        if (((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
      } else if (((unsigned char)*str) < 0xF0) {
        str++;
        if (((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
        str++;
        if (((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
      } else if (((unsigned char)*str) < 0xF8) {
        str++;
        if (((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
        str++;
        if (((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
        str++;
        if (((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
      } else
        return false;
    }
  return true;
}

bool utf8::valid(const char * str, size_t len)
{
  for (; len > 0; str++, len--)
    if (((unsigned char)*str) >= 0x80) {
      if (((unsigned char)*str) < 0xC0)
        return false;
      else if (((unsigned char)*str) < 0xE0) {
        str++;
        if (!--len || ((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
      } else if (((unsigned char)*str) < 0xF0) {
        str++;
        if (!--len || ((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
        str++;
        if (!--len || ((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
      } else if (((unsigned char)*str) < 0xF8) {
        str++;
        if (!--len || ((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
        str++;
        if (!--len || ((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
        str++;
        if (!--len || ((unsigned char)*str) < 0x80 || ((unsigned char)*str) >= 0xC0) return false;
      } else
        return false;
    }
  return true;
}

void utf8::decode(const char * str, std::u32string & decoded)
{
  decoded.clear();

  for (char32_t chr; (chr = decode(str));) decoded.push_back(chr);
}

void utf8::decode(const char * str, size_t len, std::u32string & decoded)
{
  decoded.clear();

  while (len) decoded.push_back(decode(str, len));
}

void utf8::encode(const std::u32string & str, std::string & encoded)
{
  encoded.clear();

  for (auto && chr : str) append(encoded, chr);
}

const char utf8::REPLACEMENT_CHAR;

}  // namespace unilib
}  // namespace ufal
