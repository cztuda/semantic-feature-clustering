#pragma once
#include "definition_header.h"


template<class Type, class Array, class constArray>
void objective_Lagrange(Type& fobj, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{
    
  const Type theta1 = state[0];
  const Type theta2 = state[1];
  const Type dtheta1 = state[2];
  const Type dtheta2 = state[3];
  const Type ctrl = state[4];
    
  auto Q = &constants[12];
  auto R = &constants[16];
  const auto alpha = constants[22];
  
  auto x0_tf = 0.0;
  auto x1_tf = PI;
  
  fobj = 0.0;
  fobj += Q[0]*(theta1-x0_tf)*(theta1-x0_tf) + Q[1]*(theta2-x1_tf)*(theta2-x1_tf) + Q[2]*dtheta1*dtheta1 + Q[3]*dtheta2*dtheta2;
  fobj += R[0]*ctrl*ctrl;
  
  const auto ycorr = CppAD::exp(alpha+1);
  fobj += -CppAD::exp(1-alpha*CppAD::cos(theta2))+ycorr;
};



template<class Type, class Array, class constArray>
void objective_Mayer(Type& fobj, const double tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{
  auto F = &constants[17];
  auto T_1 = constants[21];
  
  auto x0_tf = 0;
  auto x1_tf = PI;
    
  fobj = F[0]*(state_tf[0]-x0_tf)*(state_tf[0]-x0_tf) + F[1]*(state_tf[1]-x1_tf)*(state_tf[1]-x1_tf) + F[2]*state_tf[2]*state_tf[2] + F[3]*state_tf[3]*state_tf[3];
  fobj += T_1*tf;
};

