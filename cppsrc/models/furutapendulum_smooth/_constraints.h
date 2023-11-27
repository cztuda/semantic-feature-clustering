#pragma once

template<class Type, class Array, class constArray>
void constraint_NEC(Array nec, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{    
};



template<class Type, class Array, class constArray>
void constraint_NIC(Array nec, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{    
};



template<class Type, class Array, class constArray>
void constraint_implicitNBC(Array rb, Type t0, Type tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np, const unsigned int nrb)
{
    rb[0] = (1+cos(state_tf[1]));
};
