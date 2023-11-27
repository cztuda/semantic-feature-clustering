#pragma once


template<class Type, class Array, class constArray>
void objective_Lagrange(Type& fobj, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{
    const Type mu_energy = constants[2];
    const Type mu_power = constants[3];
    
    fobj = 0.0;
//     if(mu_energy > 1e-6){
        fobj +=  mu_energy*(control[0]*control[0] + control[1]*control[1] + control[2]*control[2]);
//     }
//     if(mu_power > 1e-6){
        Type sum = (Type)0.0;
        for(size_t i = 0; i < 3; ++i){
            sum += (control[i]*state[i+3])*(control[i]*state[i+3]);
        }
        fobj +=  mu_power*sum;
//     }
};



template<class Type, class Array, class constArray>
void objective_Mayer(Type& fobj, const double tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np) 
{
    const Type mu_tf = constants[1];
    
    fobj = mu_tf*tf; 
};

