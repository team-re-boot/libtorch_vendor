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

#include <bert_tokenizer/tokenizer.hpp>

namespace bert_tokenizer
{
std::string get_vocab_path(const PretrainedVocab & vocab_type)
{
  const auto vocab_file = [](const auto & vocab_type) {
    switch (vocab_type) {
      case PretrainedVocab::BERT_BASE_UNCASED:
        return "bert-base-uncased-vocab.txt";
    }
    throw std::runtime_error("Error, please specify valid vocab file.");
  }(vocab_type);
  return ament_index_cpp::get_package_share_directory("bert_tokenizer") + "/vocab/" + vocab_file;
}

BasicTokenizer::BasicTokenizer(bool doLowerCase = true) : mDoLowerCase(doLowerCase) {}

std::wstring BasicTokenizer::cleanText(const std::wstring & text) const
{
  std::wstring output;
  for (const wchar_t & cp : text) {
    if (cp == 0 || cp == 0xfffd || isControol(cp)) continue;
    if (isWhitespace(cp))
      output += L" ";
    else
      output += cp;
  }
  return output;
}

bool BasicTokenizer::isControol(const wchar_t & ch) const
{
  if (ch == L'\t' || ch == L'\n' || ch == L'\r') return false;
  auto cat = utf8proc_category(ch);
  if (cat == UTF8PROC_CATEGORY_CC || cat == UTF8PROC_CATEGORY_CF) return true;
  return false;
}

bool BasicTokenizer::isWhitespace(const wchar_t & ch) const
{
  if (ch == L' ' || ch == L'\t' || ch == L'\n' || ch == L'\r') return true;
  auto cat = utf8proc_category(ch);
  if (cat == UTF8PROC_CATEGORY_ZS) return true;
  return false;
}

bool BasicTokenizer::isPunctuation(const wchar_t & ch) const
{
  if (
    (ch >= 33 && ch <= 47) || (ch >= 58 && ch <= 64) || (ch >= 91 && ch <= 96) ||
    (ch >= 123 && ch <= 126))
    return true;
  auto cat = utf8proc_category(ch);
  if (
    cat == UTF8PROC_CATEGORY_PD || cat == UTF8PROC_CATEGORY_PS || cat == UTF8PROC_CATEGORY_PE ||
    cat == UTF8PROC_CATEGORY_PC || cat == UTF8PROC_CATEGORY_PO  //sometimes ¶ belong SO
    || cat == UTF8PROC_CATEGORY_PI || cat == UTF8PROC_CATEGORY_PF)
    return true;
  return false;
}

bool BasicTokenizer::isChineseChar(const wchar_t & ch) const
{
  if (
    (ch >= 0x4E00 && ch <= 0x9FFF) || (ch >= 0x3400 && ch <= 0x4DBF) ||
    (ch >= 0x20000 && ch <= 0x2A6DF) || (ch >= 0x2A700 && ch <= 0x2B73F) ||
    (ch >= 0x2B740 && ch <= 0x2B81F) || (ch >= 0x2B820 && ch <= 0x2CEAF) ||
    (ch >= 0xF900 && ch <= 0xFAFF) || (ch >= 0x2F800 && ch <= 0x2FA1F))
    return true;
  return false;
}

std::wstring BasicTokenizer::tokenizeChineseChars(const std::wstring & text) const
{
  std::wstring output;
  for (auto & ch : text) {
    if (isChineseChar(ch)) {
      output += L' ';
      output += ch;
      output += L' ';
    } else
      output += ch;
  }
  return output;
}

std::wstring BasicTokenizer::runStripAccents(const std::wstring & text) const
{
  //Strips accents from a piece of text.
  std::wstring nText;
  try {
    nText = convertToUnicode(normalize_nfd(convertFromUnicode(text)));
  } catch (std::bad_cast & e) {
    std::cerr << "bad_cast" << std::endl;
    return L"";
  }

  std::wstring output;
  for (auto & ch : nText) {
    auto cat = utf8proc_category(ch);
    if (cat == UTF8PROC_CATEGORY_MN) continue;
    output += ch;
  }
  return output;
}

std::vector<std::wstring> BasicTokenizer::runSplitOnPunc(const std::wstring & text) const
{
  size_t i = 0;
  bool startNewWord = true;
  std::vector<std::wstring> output;
  while (i < text.size()) {
    wchar_t ch = text[i];
    if (isPunctuation(ch)) {
      output.push_back(std::wstring(&ch, 1));
      startNewWord = true;
    } else {
      if (startNewWord) output.push_back(std::wstring());
      startNewWord = false;
      output[output.size() - 1] += ch;
    }
    i++;
  }
  return output;
}

std::vector<std::wstring> BasicTokenizer::tokenize(const std::string & text) const
{
  std::wstring nText = convertToUnicode(text);
  nText = cleanText(nText);

  nText = tokenizeChineseChars(nText);

  const std::vector<std::wstring> & origTokens = whitespaceTokenize(nText);
  std::vector<std::wstring> splitTokens;
  for (std::wstring token : origTokens) {
    if (mDoLowerCase) {
      token = tolower(token);
      token = runStripAccents(token);
    }
    const auto & tokens = runSplitOnPunc(token);
    splitTokens.insert(splitTokens.end(), tokens.begin(), tokens.end());
  }
  return whitespaceTokenize(boost::join(splitTokens, L" "));
}

WordpieceTokenizer::WordpieceTokenizer(
  const std::shared_ptr<Vocab> vocab, const std::wstring & unkToken, size_t maxInputCharsPerWord)
: mVocab(vocab), mUnkToken(unkToken), mMaxInputCharsPerWord(maxInputCharsPerWord)
{
}

std::vector<std::wstring> WordpieceTokenizer::tokenize(const std::wstring & text) const
{
  std::vector<std::wstring> outputTokens;
  for (auto & token : whitespaceTokenize(text)) {
    if (token.size() > mMaxInputCharsPerWord) {
      outputTokens.push_back(mUnkToken);
    }
    bool isBad = false;
    size_t start = 0;
    std::vector<std::wstring> subTokens;
    while (start < token.size()) {
      size_t end = token.size();
      std::wstring curSubstr;
      bool hasCurSubstr = false;
      while (start < end) {
        std::wstring substr = token.substr(start, end - start);
        if (start > 0) substr = L"##" + substr;
        if (mVocab->find(substr) != mVocab->end()) {
          curSubstr = substr;
          hasCurSubstr = true;
          break;
        }
        end--;
      }
      if (!hasCurSubstr) {
        isBad = true;
        break;
      }
      subTokens.push_back(curSubstr);
      start = end;
    }
    if (isBad)
      outputTokens.push_back(mUnkToken);
    else
      outputTokens.insert(outputTokens.end(), subTokens.begin(), subTokens.end());
  }
  return outputTokens;
}

FullTokenizer::FullTokenizer(const std::string & vocabFile, bool doLowerCase)
: mVocab(loadVocab(vocabFile)),
  mBasicTokenizer(BasicTokenizer(doLowerCase)),
  mWordpieceTokenizer(WordpieceTokenizer(mVocab))
{
  for (auto & v : *mVocab) mInvVocab[v.second] = v.first;
}

std::vector<std::wstring> FullTokenizer::tokenize(const std::string & text) const
{
  std::vector<std::wstring> splitTokens;
  for (auto & token : mBasicTokenizer.tokenize(text))
    for (auto & subToken : mWordpieceTokenizer.tokenize(token)) splitTokens.push_back(subToken);
  return splitTokens;
}

std::vector<size_t> FullTokenizer::convertTokensToIds(const std::vector<std::wstring> & text) const
{
  std::vector<size_t> ret(text.size());
  for (size_t i = 0; i < text.size(); i++) {
    ret[i] = (*mVocab)[text[i]];
  }
  return ret;
}

// std::string get_vocab_path()
// {

// }

}  // namespace bert_tokenizer
