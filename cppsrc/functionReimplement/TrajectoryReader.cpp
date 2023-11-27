
#include "TrajectoryReader.h"


#include <iostream>
#include <sstream>
#include <cmath>
#include <cctype>

#include "NormalizedPolynom.h"
#include "piecewisefunctiondata.h"

using namespace ofc::functionLib;
using namespace std;


std::string toLowercase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c){ return std::tolower(c); });
    return s;
}


bool TrajectoryReader::read(std::istream & stream, PiecewiseFunction** trajectory, PiecewiseFunctionData* dataExport){
  if(!stream)return false;

  // read dimension of the trajectory
  int dim;
  stream >> dim;
  string line1;
  string line2;
  getline(stream,line1);
  getline(stream,line2);
  
  int nParameter;
  char areadj;
  std::istringstream(line1) >> areadj;
  std::istringstream(line1) >> nParameter;
  
  int nPhases = 0;
  std::istringstream (line2) >> nPhases;
  
  // read the number of knots
  uint knots = 0;
  vector<uint> gridpoints;
  int ntmp;
  for(int i = 0; i < nPhases; i++){
    ntmp = 0;
    getline(stream,line1);
    std::istringstream (line1) >> ntmp;
    gridpoints.push_back(ntmp);
    knots += ntmp;
  }
  //
  
  // read the interval over which the function is defined
  Interval d;
  stream >> d.a; stream >> d.b;
  getline(stream,line1);
  // read all names
  vector<std::string> names;
  for(int i=0; i < dim; i++){
    string name;
    getline(stream,name);
    names.push_back(name);
  }
  // read all doubles
  vector<double> doubles;
  while(stream){
    double tmp;
    stream >> tmp;
    if(stream.fail()){
      std::string currentstr;
      stream.clear();
      stream >> currentstr;
      if(!toLowercase(currentstr).compare("nan")){
	tmp = NAN;
      }
      else if(currentstr.size() == 0) {
	// ignore
      }
      else {
	return false;
      }
    }
    doubles.push_back(tmp);
  }
  //remove last double because it is invalid. (hopefully always)
  doubles.pop_back();
  //further remove all parameter from doubles and store them in an extra vector
  vector<double> parameter;
  parameter.resize(nParameter);
  for(int i = 0; i < nParameter && doubles.size() > 0; i++){
    parameter[nParameter-1-i] = doubles.back();
    doubles.pop_back();
  }
  
  // export the data from the file if requested
  if(dataExport != 0){
    dataExport->n = dim;
    dataExport->np = nParameter;
    dataExport->nphases = nPhases;
    dataExport->estadj = areadj;
    dataExport->t0 = d.a;
    dataExport->tf = d.b;
    dataExport->ngridpts = gridpoints;
    dataExport->parameter = parameter;
    dataExport->names = names;
    dataExport->doubles = doubles;
    if(trajectory == 0) return true;
  }
  
  
  // time entries
  int order=3;
  // calculate the number of doubles needed for cubic or linear piecwise function
  int cubic = (dim*(order+1)+2)*(knots-1)+dim+1;
  int linear = (dim+1)*knots;
  PiecewiseFunction * piecewise= new PiecewiseFunction(dim);
  
  if(doubles.size()==cubic){
    if(!  parseCubicPiecewiseFunction(piecewise,doubles,dim)){
      delete piecewise;
      return false;
    }
  }
  else if(doubles.size()==linear){
    if(!  parseLinearPiecewiseFunction(piecewise,doubles,dim)){
      delete piecewise;
      return false;
    }
  } 
  else {
    delete piecewise;
    return false;
  }
  
  if(trajectory){
    *trajectory=piecewise;
  }
  else {
    delete piecewise;
    return false;
  }
  return true;
}


bool TrajectoryReader::parseCubicPiecewiseFunction(PiecewiseFunction * piecewise, std::vector<Real> & doubles, uint dim){
  std::vector<Real> coefficients;
  Real t_k[2];

  uint stride = dim*4+2;
  uint lastStride = dim+1;
  coefficients.reserve(stride-2);
  NormalizedPolynom * f_k=0;
  size_t offset;
  
  // large loop to create piecewise function
  for(offset = 0; offset < doubles.size()-lastStride; offset+=stride){
    t_k[0] = doubles[offset+0];
    t_k[1] = doubles[offset+1];
    int coeffOffset_k = offset+2;
    
    // sort coefficients:
    for(int nn = 0; nn < 4; nn++){
      for(int j=0; j < dim; j++){
	coefficients.push_back(doubles[coeffOffset_k + j*4 + nn]);
      }
    }

    // create new piece:
    f_k = new NormalizedPolynom(dim, &coefficients, Interval(t_k[0], t_k[1]));
    piecewise->add(t_k[0], f_k);
    
    coefficients.clear();
  }
  
  // create and add last piece:
  t_k[0] = doubles[offset]; offset++;
  for(; offset < doubles.size(); offset++){
    coefficients.push_back(doubles[offset]);
  }
  f_k = new NormalizedPolynom(dim, &coefficients, Interval(t_k[0], t_k[0]));
  piecewise->add(t_k[0], f_k);
  
  return true;
}



bool TrajectoryReader::parseLinearPiecewiseFunction(PiecewiseFunction * piecewise, std::vector<Real> & doubles, uint dim){
  // create all sample points and times
  int n = doubles.size()/(1+dim);
  std::vector<Real> t;
  std::vector<Real> u;
  t.reserve(n);
  u.reserve(dim*n);
  
  n = 0;
  for(std::vector<Real>::iterator it = doubles.begin(); it < doubles.end(); ){
    t.push_back(*it); it++;
    u.insert(u.end(), it, it+dim);
    it += dim;
    n++;
  }
  
  NormalizedPolynom * f_k=0;

  auto itu = u.begin();
  for(size_t k=0; k < n-1;k++){
    Interval interval(t[k],t[k+1]); // create interval
    std::vector<Real> coeffs; // create coefficient vector
    coeffs.reserve(dim*2);
    coeffs.insert(coeffs.end(), itu, itu+dim);
    for(size_t iiu = 0; iiu < dim; iiu++){
      coeffs.push_back((*(itu+dim)) - (*itu));
      itu++;
    }
    
    f_k = new NormalizedPolynom(dim, &coeffs, interval);
    piecewise->add(t[k],f_k);
  }
  
  Interval interval(t[n-1],t[n-1]); // create interval
  std::vector<Real> coeffs; // create coefficient vector
  coeffs.reserve(dim);
  coeffs.insert(coeffs.end(), itu, itu+dim);
  
  f_k = new NormalizedPolynom(dim, &coeffs, interval);
  piecewise->add(t[n-1],f_k);
  
  return true;
}



