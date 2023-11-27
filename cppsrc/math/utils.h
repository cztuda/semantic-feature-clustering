#pragma once


namespace ofc::math {


template<typename T>
T pow(T base, int exp){
  T result = 1;
  while(exp){
    if(exp & 1)
      result *= base;
    exp >>= 1;
    base *= base;
  }
  return result;
}




}
