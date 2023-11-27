#pragma once

#include <cmath>

template<class Type, class Array, class constArray>
void objective_Lagrange(Type& fobj, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{
  auto Q = &constants[14];
  auto R = &constants[18];
  
  auto x0_tf = 0;
  auto x1_tf = 0;
  
  fobj = 0.0;
  fobj += Q[0]*(state[0]-x0_tf)*(state[0]-x0_tf) + Q[1]*(state[1]-x1_tf)*(state[1]-x1_tf) + Q[2]*state[2]*state[2] + Q[3]*state[3]*state[3];
  fobj += R[0]*control[0]*control[0];
  
  const auto alpha = constants[24];
  const auto ycorr = CppAD::exp(alpha+1);
  fobj += -CppAD::exp(-alpha*CppAD::cos(state[1])+1)+ycorr;
};



template<class Type, class Array, class constArray>
void objective_Mayer(Type& fobj, const double tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{
  auto F = &constants[19];
  auto T_1 = constants[23];
  
  auto x0_tf = 0;
  auto x1_tf = 0;
    
  fobj = F[0]*(state_tf[0]-x0_tf)*(state_tf[0]-x0_tf) + F[1]*(state_tf[1]-x1_tf)*(state_tf[1]-x1_tf) + F[2]*state_tf[2]*state_tf[2] + F[3]*state_tf[3]*state_tf[3];
  fobj += T_1*tf;
};

