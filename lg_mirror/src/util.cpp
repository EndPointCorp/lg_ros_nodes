#include "util.h"

#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

const std::size_t BUF_SZ = 4096;

namespace util {

// http://stackoverflow.com/a/478960
std::string exec(const char* cmd) {
  char buffer[BUF_SZ];
  std::string result = "";
  FILE* pipe = popen(cmd, "r");
  if (!pipe) throw std::runtime_error("popen() failed!");
  try {
    while (!feof(pipe)) {
      if (fgets(buffer, BUF_SZ, pipe) != NULL) {
        result += buffer;
      }
    }
  } catch (...) {
    pclose(pipe);
    throw;
  }
  pclose(pipe);
  return result;
}

} // namespace util
