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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

std::vector<std::string> whitespace_tokenize(std::string text);

std::map<std::string, int> read_vocab(const char * filename);

class BasicTokenizer
{
public:
  bool do_lower_case_;
  std::vector<std::string> never_split_;

  BasicTokenizer(
    bool do_lower_case = false,
    std::vector<std::string> never_split = {"[UNK]", "[SEP]", "[PAD]", "[CLS]", "[MASK]"})
  {
    do_lower_case_ = do_lower_case;
    never_split_ = never_split;
  }

  std::string _clean_text(std::string text) const;

  std::vector<std::string> _run_split_on_punc(std::string text) const;

  std::string _run_strip_accents(std::string text) const;

  std::string _tokenize_chinese_chars(std::string text) const;

  std::string utf8chr(int cp) const;

  bool _is_chinese_char(int cp) const;

  std::vector<std::string> tokenize(const std::string & text) const;

  void truncate_sequences(
    std::vector<std::string> & textA, std::vector<std::string> & textB,
    const char * truncation_strategy, size_t max_seq_length);
};

class WordpieceTokenizer
{
public:
  std::map<std::string, int> vocab_;
  std::string unk_token_;
  int max_input_chars_per_word_;

  WordpieceTokenizer(){};

  WordpieceTokenizer(
    std::map<std::string, int> vocab, std::string unk_token = "[UNK]",
    int max_input_chars_per_word = 100)
  {
    vocab_ = vocab;
    unk_token_ = unk_token;
    max_input_chars_per_word_ = max_input_chars_per_word;
  }

  void add_vocab(std::map<std::string, int> vocab);

  std::vector<std::string> tokenize(const std::string & text) const;
};

class BertTokenizer
{
public:
  enum class PretrainedVocabFile { BERT_BASE_UNCASED };
  std::map<std::string, int> vocab;
  std::map<int, std::string> ids_to_tokens;
  bool do_lower_case_;
  bool do_basic_tokenize_;
  size_t maxlen_;
  BasicTokenizer basic_tokenizer;
  WordpieceTokenizer wordpiece_tokenizer;

  explicit BertTokenizer(const PretrainedVocabFile & vocab)
  {
    const auto vocab_file = [](const auto & vocab) {
      switch (vocab) {
        case PretrainedVocabFile::BERT_BASE_UNCASED:
          return "bert-base-uncased-vocab.txt";
      }
      throw std::runtime_error("Error, please specify valid vocab file.");
    }(vocab);
    add_vocab(
      std::string(
        ament_index_cpp::get_package_share_directory("bert_tokenizer") + "/vocab/" + vocab_file)
        .c_str());
  }

  explicit BertTokenizer(
    const char * vocab_file, bool do_lower_case = false, int max_len = 512,
    bool do_basic_tokenize = true,
    std::vector<std::string> /*never_split*/ = {"[UNK]", "[SEP]", "[PAD]", "[CLS]", "[MASK]"})
  {
    vocab = read_vocab(vocab_file);
    for (std::map<std::string, int>::iterator i = vocab.begin(); i != vocab.end(); ++i)
      ids_to_tokens[i->second] = i->first;
    do_basic_tokenize_ = do_basic_tokenize;
    do_lower_case_ = do_lower_case;
    wordpiece_tokenizer.add_vocab(vocab);
    maxlen_ = max_len;
  }

  void encode(
    std::string textA, std::string textB, std::vector<float> & input_ids,
    std::vector<float> & input_mask, std::vector<float> & segment_ids, size_t max_seq_length = 512,
    const char * truncation_strategy = "longest_first") const;

private:
  void add_vocab(const char * vocab_file);
  std::vector<std::string> tokenize(const std::string & text) const;
  std::vector<float> convert_tokens_to_ids(std::vector<std::string> tokens) const;
};