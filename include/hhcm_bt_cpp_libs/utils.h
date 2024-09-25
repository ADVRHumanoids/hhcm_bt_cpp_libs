#ifndef HHCM_BT_CPP_LIBS_UTILS_H
#define HHCM_BT_CPP_LIBS_UTILS_H

#include "behaviortree_cpp/behavior_tree.h"
#include <cstring>

namespace hhcm_bt {
namespace utils {

int BTCppLibraryVersionNumber()
{
  static int number = -1;
  if(number == -1)
  {
    auto const parts = BT::splitString(BTCPP_LIBRARY_VERSION, '.');
    number = std::stoi(std::string(parts[0])) * 10000 +
             std::stoi(std::string(parts[1])) * 100 + std::stoi(std::string(parts[2]));
  }
  return number;
}

} // namespace utils
} // namespace hhcm_bt_cpp_libs

#endif // HHCM_BT_CPP_LIBS_UTILS_H