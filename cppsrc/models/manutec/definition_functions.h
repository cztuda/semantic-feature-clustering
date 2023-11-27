#pragma once

void getDimensions(unsigned int& nx, unsigned int& nu, unsigned int& nc, unsigned int& np)
{nx = DIM_X; nu = DIM_U; np = DIM_P; nc = DIM_C;}

template<class Type, class Array, class constArray>
void objective_Lagrange(Type& fobj, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np);

template<class Type, class Array, class constArray>
void objective_Mayer(Type& fobj, const double tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np);

template<class Type, class Array, class constArray>
void dynamics(Array f, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np, const unsigned int mode);

template<class Type, class Array, class constArray>
void constraint_NEC(Array nec, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np);

template<class Type, class Array, class constArray>
void constraint_NIC(Array nec, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np);

template<class Type, class Array, class constArray>
void constraint_NBC(int i, Type t0, Type tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np);

#include "_objective.h"
#include "_dynamics.h"
#include "_constraints.h"
