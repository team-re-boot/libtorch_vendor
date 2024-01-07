// Copyright 2024 Team Re-Boot. All rights reserved.
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

/// This code comes from https://gist.github.com/luistung/ace4888cf5fd1bad07844021cb2c7ecf

#include <utf8proc.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace bert_tokenizer
{
const std::wstring stripChar = L" \t\n\r\v\f";
using Vocab = std::unordered_map<std::wstring, size_t>;
using InvVocab = std::unordered_map<size_t, std::wstring>;

std::string normalize_nfd(const std::string & s)
{
  std::string ret;
  char * result = (char *)utf8proc_NFD((unsigned char *)s.c_str());
  if (result) {
    ret = std::string(result);
    free(result);
    result = NULL;
  }
  return ret;
}

bool isStripChar(const wchar_t & ch) { return stripChar.find(ch) != std::wstring::npos; }

std::wstring strip(const std::wstring & text)
{
  std::wstring ret = text;
  if (ret.empty()) return ret;
  size_t pos = 0;
  while (pos < ret.size() && isStripChar(ret[pos])) pos++;
  if (pos != 0) ret = ret.substr(pos, ret.size() - pos);
  pos = ret.size() - 1;
  while (pos != (size_t)-1 && isStripChar(ret[pos])) pos--;
  return ret.substr(0, pos + 1);
}

std::vector<std::wstring> split(const std::wstring & text)
{
  std::vector<std::wstring> result;
  boost::split(result, text, boost::is_any_of(stripChar));
  return result;
}

std::vector<std::wstring> whitespaceTokenize(const std::wstring & text)
{
  std::wstring rtext = strip(text);
  if (rtext.empty()) return std::vector<std::wstring>();
  return split(text);
}

std::wstring convertToUnicode(const std::string & text)
{
  size_t i = 0;
  std::wstring ret;
  while (i < text.size()) {
    wchar_t codepoint;
    utf8proc_ssize_t forward = utf8proc_iterate(
      (utf8proc_uint8_t *)&text[i], text.size() - i, (utf8proc_int32_t *)&codepoint);
    if (forward < 0) return L"";
    ret += codepoint;
    i += forward;
  }
  return ret;
}

std::string convertFromUnicode(const std::wstring & wText)
{
  char dst[64];
  std::string ret;
  for (auto ch : wText) {
    utf8proc_ssize_t num = utf8proc_encode_char(ch, (utf8proc_uint8_t *)dst);
    if (num <= 0) return "";
    ret += std::string(dst, dst + num);
  }
  return ret;
}

std::wstring tolower(const std::wstring & s)
{
  std::wstring ret(s.size(), L' ');
  for (size_t i = 0; i < s.size(); i++) {
    ret[i] = utf8proc_tolower(s[i]);
  }
  return ret;
}

std::shared_ptr<Vocab> loadVocab(const std::string & vocabFile)
{
  std::shared_ptr<Vocab> vocab(new Vocab);
  size_t index = 0;
  std::ifstream ifs(vocabFile, std::ifstream::in);
  std::string line;
  while (getline(ifs, line)) {
    std::wstring token = convertToUnicode(line);
    if (token.empty()) break;
    token = strip(token);
    (*vocab)[token] = index;
    index++;
  }
  return vocab;
}

enum class PretrainedVocab { BERT_BASE_UNCASED };

std::string get_vocab_path(const PretrainedVocab & vocab_type);

class BasicTokenizer
{
public:
  BasicTokenizer(bool doLowerCase);
  std::vector<std::wstring> tokenize(const std::string & text) const;

private:
  std::wstring cleanText(const std::wstring & text) const;
  bool isControol(const wchar_t & ch) const;
  bool isWhitespace(const wchar_t & ch) const;
  bool isPunctuation(const wchar_t & ch) const;
  bool isChineseChar(const wchar_t & ch) const;
  std::wstring tokenizeChineseChars(const std::wstring & text) const;
  bool isStripChar(const wchar_t & ch) const;
  std::wstring strip(const std::wstring & text) const;
  std::vector<std::wstring> split(const std::wstring & text) const;
  std::wstring runStripAccents(const std::wstring & text) const;
  std::vector<std::wstring> runSplitOnPunc(const std::wstring & text) const;

  bool mDoLowerCase;
};

class WordpieceTokenizer
{
public:
  WordpieceTokenizer(
    std::shared_ptr<Vocab> vocab, const std::wstring & unkToken = L"[UNK]",
    size_t maxInputCharsPerWord = 200);
  std::vector<std::wstring> tokenize(const std::wstring & text) const;

private:
  std::shared_ptr<Vocab> mVocab;
  std::wstring mUnkToken;
  size_t mMaxInputCharsPerWord;
};

class FullTokenizer
{
public:
  FullTokenizer(const std::string & vocabFile, bool doLowerCase = true);
  std::vector<std::wstring> tokenize(const std::string & text) const;
  std::vector<size_t> convertTokensToIds(const std::vector<std::wstring> & text) const;

private:
  std::shared_ptr<Vocab> mVocab;
  InvVocab mInvVocab;
  std::string mVocabFile;
  bool mDoLowerCase;
  BasicTokenizer mBasicTokenizer;
  WordpieceTokenizer mWordpieceTokenizer;
};
}  // namespace bert_tokenizer
