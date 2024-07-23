#pragma once

#include <string>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


bool ExistsFile(const std::string& path) {
  return boost::filesystem::is_regular_file(path);
}

bool ExistsDir(const std::string& path) {
  return boost::filesystem::is_directory(path);
}

bool ExistsPath(const std::string& path) {
  return boost::filesystem::exists(path);
}


void CreateDirIfNotExists(const std::string& path) {
  if (!ExistsDir(path)) {
    boost::filesystem::create_directory(path);
  }
}
std::string GetParentDir(const std::string& path) {
  return boost::filesystem::path(path).parent_path().string();
}
template <typename... T>
std::string JoinPaths(T const&... paths) {
  boost::filesystem::path result;
  int unpack[]{0, (result = result / boost::filesystem::path(paths), 0)...};
  static_cast<void>(unpack);
  return result.string();
}

static bool IsNotWhiteSpace(const int character) {
  return character != ' ' && character != '\n' && character != '\r' &&
        character != '\t';
}

void StringLeftTrim(std::string* str) {
  str->erase(str->begin(),
            std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void StringRightTrim(std::string* str) {
  str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
            str->end());
}
std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim) {
  std::vector<std::string> elems;
  boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
  return elems;
}
void StringTrim(std::string* str) {
  StringLeftTrim(str);
  StringRightTrim(str);
}

template <typename T>
T ReverseBytes(const T& data) {
  T data_reversed = data;
  std::reverse(reinterpret_cast<char*>(&data_reversed),
              reinterpret_cast<char*>(&data_reversed) + sizeof(T));
  return data_reversed;
}

inline bool IsLittleEndian() {
#ifdef BOOST_BIG_ENDIAN
  return false;
#else
  return true;
#endif
}

template <typename T>
T LittleEndianToNative(const T x) {
  if (IsLittleEndian()) {
    return x;
  } else {
    return ReverseBytes(x);
  }
}

template <typename T>
T ReadBinaryLittleEndian(std::istream* stream) {
  T data_little_endian;
  stream->read(reinterpret_cast<char*>(&data_little_endian), sizeof(T));
  return LittleEndianToNative(data_little_endian);
}

template <typename T>
void ReadBinaryLittleEndian(std::istream* stream, std::vector<T>* data) {
  for (size_t i = 0; i < data->size(); ++i) {
    (*data)[i] = ReadBinaryLittleEndian<T>(stream);
  }
}
template <typename T>
std::string VectorToCSV(const std::vector<T>& values) {
  std::string string;
  for (const T value : values) {
    string += std::to_string(value) + " ";
  }
  return string.substr(0, string.length() - 2);
}
