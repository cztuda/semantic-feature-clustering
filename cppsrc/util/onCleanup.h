#pragma once

#include <functional>

namespace ofc{

class onCleanup {
public:
  onCleanup(const std::function<void()>& f) : fun(f)
  {};
  
  ~onCleanup() {
    fun();
  }

private:
  std::function<void()> fun;

};


}
