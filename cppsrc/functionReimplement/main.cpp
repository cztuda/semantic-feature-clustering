#include <iostream>
#include <fstream>

#include "../math/definitions.h"
#include "Interval.h"
#include "Function.h"
#include "Polynom.h"
#include "PiecewiseFunction.h"
#include "TrajectoryReader.h"

using namespace ofc::functionLib;

#define printArray(X) for(int i = 0; i < sizeof(X)/sizeof(X[0])-1; i++){ std::cout << X[i] << ", ";} std::cout << X[sizeof(X)/sizeof(X[0])-1] << std::endl; 

void testPolynomial(){
  
  Polynom p1(3, 1);
  Polynom p2(5, 2);
  
  Real x1[] = {1};
  Real x2[] = {1, 1.5, 2};
  Real x3[] = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  
  Real res11[3];
  Real res12[3*3];
  Real res13[3*11];
  
  p1.evaluate(res11, x1);
  p1.evaluate(res12, x2, 3);
  p1.evaluate(res13, x3, 11);
  std::cout << "res11: "; printArray(res11);
  std::cout << "res12: "; printArray(res12);
  std::cout << "res13: "; printArray(res13);
  
  std::vector<Real> coeff11;
  coeff11.push_back(1);
  coeff11.push_back(1.5);
  coeff11.push_back(3);
  std::vector<Real> coeff12;
  coeff12.push_back(1);
  coeff12.push_back(2);
  coeff12.push_back(3);
  std::vector<Real> coeff13;
  coeff13.push_back(2);
  coeff13.push_back(-1.5);
  coeff13.push_back(1);
  
  std::cout << "p1 with coeff 1: " << std::endl;
  p1.setCoefficients(&coeff11);
  p1.evaluate(res11, x1);
  p1.evaluate(res12, x2, 3);
  p1.evaluate(res13, x3, 11);
  

  std::cout << "res11: "; printArray(res11);
  std::cout << "res12: "; printArray(res12);
  std::cout << "res13: "; printArray(res13);
  
  Real coeffs21[] = {1, 1.5, 1, 2, 1,   2, 2.2, 3, 1, 5};
  std::vector<Real> coeff21(&coeffs21[0], &coeffs21[10]);
    
  Real res21[5];
  Real res22[5*3];
  Real res23[5*11];
  
  std::cout << "p2 with coeff 1: " << std::endl;
  p2.setCoefficients(&coeff21);
  p2.evaluate(res21, x1);
  p2.evaluate(res22, x2, 3);
  p2.evaluate(res23, x3, 11);
  

  std::cout << "res21: "; printArray(res21);
  std::cout << "res22: "; printArray(res22);
  std::cout << "res23: "; printArray(res23);
  
  Polynom p3(3, 4);
  Real coeffs31[] = {1, 2, 1.5,   2.2, 4, 3.5,   1, 1.1, 1.3,   3.5, 2.5, 4.5};
  std::vector<Real> coeff31(&coeffs31[0], &coeffs31[12]);
  
  Real x4[] = {1.5, 2};
  Real res31[3];
  Real res32[6];
  p3.setCoefficients(&coeff31);
  
  std::cout << "p3, normal evaluation: " << std::endl;
  p3.evaluate(res31, x1);
  p3.evaluate(res32, x4, 2);
  std::cout << "res31: "; printArray(res31);
  std::cout << "res32: "; printArray(res32);
  
  std::cout << "p3, 1. derivative evaluation: " << std::endl;
  p3.evaluateDerivative(res31, x1, 1, 1);
  p3.evaluateDerivative(res32, x4, 2, 1);
  std::cout << "res31: "; printArray(res31);
  std::cout << "res32: "; printArray(res32);
  
  std::cout << "p3, 2. derivative evaluation: " << std::endl;
  p3.evaluateDerivative(res31, x1, 1, 2);
  p3.evaluateDerivative(res32, x4, 2, 2);
  std::cout << "res31: "; printArray(res31);
  std::cout << "res32: "; printArray(res32);
  
  std::cout << "p3, 3. derivative evaluation: " << std::endl;
  p3.evaluateDerivative(res31, x1, 1, 3);
  p3.evaluateDerivative(res32, x4, 2, 3);
  std::cout << "res31: "; printArray(res31);
  std::cout << "res32: "; printArray(res32);
  
  std::cout << "p3, 4. derivative evaluation: " << std::endl;
  p3.evaluateDerivative(res31, x1, 1, 4);
  p3.evaluateDerivative(res32, x4, 2, 4);
  std::cout << "res31: "; printArray(res31);
  std::cout << "res32: "; printArray(res32);
}

void testPiecewiseFunction(){
  std::vector<Real> coeffs1 = {1, 1, 3, 3};
  std::vector<Real> coeffs2 = {0, 1, 2, -1};
  std::vector<Real> coeffs3 = {2, 1, -1, -1};
  Polynom* p1 = new Polynom(2, &coeffs1);
  Polynom* p2 = new Polynom(2, &coeffs2);
  Polynom* p3 = new Polynom(2, &coeffs3);
  
  
  PiecewiseFunction f(2);
  f.add(0, p1);
  f.add(1.2, p2);
  f.add(2, p3);
  
  Real res[2];
  Real val[] = {0.8};
  f.evaluate(res, val);
  printArray(res);
  
  val[0] = 1.5;
  f.evaluate(res, val);
  printArray(res);
  
  val[0] = 0;
  f.evaluate(res, val);
  printArray(res);
  
  val[0] = 2;
  f.evaluate(res, val);
  printArray(res);
  
  val[0] = -1;
  f.evaluate(res, val);
  printArray(res);
  
  val[0] = 3;
  f.evaluate(res, val);
  printArray(res);
}

void testTrajectoryReader(){
  TrajectoryReader tr;
  
  std::ifstream stream("GDATU");
  PiecewiseFunction* fun;
  tr.read(stream, &fun);
  stream.close();
  
  Real result[3];
  Real x[] = {0.0};
  fun->evaluate(result, x);
  printArray(result);
  
  
  x[0] = 0.3;
  fun->evaluate(result, x);
  printArray(result);
  
  
  Interval I = fun->getDefinitionInterval();
  x[0] = I.b;
  fun->evaluate(result, x);
  printArray(result);
  
  delete fun;
  stream.open("GDATX");
  tr.read(stream, &fun);
  stream.close();
  std::cout << "cubic reader:" << std::endl;
  
  Real result2[7];
  x[0] = 0;
  fun->evaluate(result2, x);
  printArray(result2);
  
  x[0] = 0.3;
  fun->evaluate(result2, x);
  printArray(result2);
  
  I = fun->getDefinitionInterval();
  x[0] = I.b;
  fun->evaluate(result2, x);
  printArray(result2);
  
  delete fun;
}


void testSerialization(){
  Polynom p1(3, 1);
  
  Real x2[] = {1, 1.5, 2};
  
  Real res12[3*3];
  
  
  std::vector<Real> coeff11 = {1, 1.5, 3};  
  p1.setCoefficients(&coeff11);
  p1.evaluate(res12, x2, 3);
  std::cout << "res12: "; printArray(res12);
  
  std::ofstream stream("serialtest_01.txt");
  Function::serialize(&p1, stream);
  stream.close();
  
  std::ifstream ss("serialtest_01.txt");
  std::string type;
  ss >> type;
  Polynom* f;
  Polynom::unserialize(&f, ss);
//   Polynom* f = dynamic_cast<Polynom*>(serialization::unserializeFunction(type, ss));
  ss.close();
  f->evaluate(res12, x2, 3);
  std::cout << "res12: "; printArray(res12);
  std::cout  << std::endl;
  delete f;
  
  
  ////////////////
  
  TrajectoryReader tr;
  std::ifstream s1("GDATU");
  PiecewiseFunction* fun;
  tr.read(s1, &fun);
  s1.close();
  
  Real result[3];
  Real x[] = {0.0};
  fun->evaluate(result, x);
  printArray(result);
  x[0] = 0.3;
  fun->evaluate(result, x);
  printArray(result);
  x[0] = fun->getDefinitionInterval().b;
  fun->evaluate(result, x);
  printArray(result);
  
  std::ofstream s2("serialtest_02.txt");
  Function::serialize(fun, s2);
  s2.close();
  
  std::ifstream s3("serialtest_02.txt");
  s3 >> type;
  PiecewiseFunction* f2;
  PiecewiseFunction::unserialize(&f2, s3);
//   PiecewiseFunction* f2 = dynamic_cast<PiecewiseFunction*>(serialization::unserializeFunction(type, s3));
  s3.close();
  
  
  x[0] = 0.0;
  f2->evaluate(result, x);
  printArray(result);
  x[0] = 0.3;
  f2->evaluate(result, x);
  printArray(result);
  x[0] = f2->getDefinitionInterval().b;
  f2->evaluate(result, x);
  printArray(result);
  
  
  delete f2;
}


int main(){
    std::cout << "Started testsuit ..." << std::endl;
  
//     std::istringstream ss(std::string("teststream er 14 test"));
//     std::string tmp;
//     int t;
//     ss >> tmp;
//     std::cout << tmp << " (" << (ss.rdstate() & std::istringstream::failbit) << ")" << std::endl;
//     ss.clear(); ss >> t; 
//     std::cout << " " << t << " (" << (ss.rdstate() & std::istringstream::failbit) << ")" << std::endl;
//     ss.clear(); ss >> t;
//     std::cout << " " << t<< " (" << (ss.rdstate() & std::istringstream::failbit) << ")" << std::endl;
//     ss.clear(); ss >> tmp;
//     std::cout << " " << tmp << " (" << (ss.rdstate() & std::istringstream::failbit) << ")" << std::endl;
//     
    
    testPolynomial();
    std::cout << "\n===========================================================================================\n" << std::endl;
    testPiecewiseFunction();
    std::cout << "\n===========================================================================================\n" << std::endl;
    testTrajectoryReader();
    std::cout << "\n===========================================================================================\n" << std::endl;
    testSerialization();
    
    std::cout << "Terminated testsuit." << std::endl;
    return 0;
}

