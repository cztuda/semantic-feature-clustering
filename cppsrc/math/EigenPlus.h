
#ifndef EIGENPLUS_H
#define EIGENPLUS_H

#include <iostream>
#include <cctype>
#include <cstring>
#include <vector>

#include <Eigen/Core>
namespace eigenplus {


enum SerializationError {
  DEFAULT=-1,
  UNEXPECTED_NUMBER_OF_ROWS=-2,
  UNEXPECTED_NUMBER_OF_COLUMNS=-3,
  UNEXPECTED_SYMBOL=-4,
  MORE_DATA_THAN_EXPECTED=-5,
  LESS_DATA_THAN_EXPECTED=-6,
};

template <typename Derived>
void writeEigenMatrix(std::ostream& s, const Eigen::MatrixBase<Derived>& m, bool writeDimension=true)
{
  if(writeDimension){
    s << "<" << m.rows() << "," << m.cols();
  }
  Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, "|", "|", "", "", ">", "");
  s << m.format(CommaInitFmt) << " ";
}

inline void streamAdvanceToEnd(std::istream& s){
  while(!isspace(s.get()) && !s.eof()){}
}

const int MAX_NUM_LEN = 30;

inline double readDouble(std::istream& s){
  char tmp[MAX_NUM_LEN+2];
  size_t i;
  for(i = 0; i < MAX_NUM_LEN && !s.eof(); i++){
    tmp[i] = s.get();
    if(tmp[i] == '|' || isspace(tmp[i])) {
      s.unget();
      break;
    }
  }
  tmp[i] = '\0';
  return atof(tmp);
}
inline double readInt(std::istream& s){
  char tmp[MAX_NUM_LEN+2];
  size_t i;
  for(i = 0; i < MAX_NUM_LEN && !s.eof(); i++){
    tmp[i] = s.get();
    if(!isdigit(tmp[i])) {
      s.unget();
      break;
    }
  }
  tmp[i] = '\0';
  return atoi(tmp);
}

template <typename Derived>
int readEigenMatrix(std::istream& s, Eigen::MatrixBase<Derived>& m, int rows = -1, int cols = -1)
{
  char c = s.get();
  while(isspace(c)){ 
    if(s.eof()) {break;}
    c = s.get();
  }
  if(c == '<'){
    // read size here (and check if it matches given rows and cols)
    int tmp = readInt(s);
    if(rows >= 0){
      if(tmp != rows){
	streamAdvanceToEnd(s);
	return SerializationError::UNEXPECTED_NUMBER_OF_ROWS;
      }
    }
    else {
      rows = tmp;
    }
    c = s.get();
    if(c != ','){
      streamAdvanceToEnd(s);
      return SerializationError::UNEXPECTED_SYMBOL;
    }
    tmp = readInt(s);
    if(cols >= 0){
      if(tmp != cols){
	streamAdvanceToEnd(s);
	return SerializationError::UNEXPECTED_NUMBER_OF_COLUMNS;
      }
    }
    else {
      cols = tmp;
    }
    c = s.get();
  }
  if(c == '>'){
    // do nothing here
  }
  else {
    streamAdvanceToEnd(s);
    return SerializationError::UNEXPECTED_SYMBOL;
  }
  
  bool varlen = false;
  int len = rows*cols;
  if(len < 0 || rows < 0){
    varlen = true;
  }
  else {
    m.derived().resize(rows, cols);
    if(m.derived().rows() < rows) return SerializationError::UNEXPECTED_NUMBER_OF_ROWS;
    if(m.derived().cols() < cols) return SerializationError::UNEXPECTED_NUMBER_OF_COLUMNS;
  }
  if(!varlen){
    int i = 0;
    while(i < len){
      // read entry:
      m(i) = readDouble(s);
      // check delimiter:
      c = s.get();
      if(c == '|'){
	if(i+1==len && !varlen){
	  // more data than expected
	  streamAdvanceToEnd(s);
	  return SerializationError::MORE_DATA_THAN_EXPECTED;
	}
      }
      else if(s.eof() || isspace(c)){
	if(varlen){
	  break;
	}
	if(i+1<len && !varlen){
	  // less data than expected
	  streamAdvanceToEnd(s);
	  return SerializationError::LESS_DATA_THAN_EXPECTED;
	}
      }
      else {
	// unexpected character
	  streamAdvanceToEnd(s);
	  return SerializationError::UNEXPECTED_SYMBOL;
      }
      
      i++;
      if(varlen && i == len){
	len *= 10;
	m.derived().resize(len, 1);
      }
    }
    return i;
  }
  else {
    std::vector<double> buf;
    buf.reserve(10);
    while(!s.eof()){
      buf.push_back(readDouble(s));
      c = s.get();
      if(isspace(c)) break;
      else if(c != '|'){
	// unexpected character
	streamAdvanceToEnd(s);
	return SerializationError::UNEXPECTED_SYMBOL;
      }
    }
    
    m.derived().resize(buf.size(),1);
    if(m.derived().cols() != 1) {
      return SerializationError::UNEXPECTED_NUMBER_OF_COLUMNS;
    }
    int len = m.derived().rows()*m.derived().cols();
    if(len < buf.size()) {
      return SerializationError::MORE_DATA_THAN_EXPECTED;
    }
    else if(buf.size() < len){
      len = buf.size();
      return SerializationError::LESS_DATA_THAN_EXPECTED;
    }
    memcpy(m.derived().data(), buf.data(), sizeof(double)*len);
    return len;
  }
    
}


}
#endif // EIGENPLUS_H
