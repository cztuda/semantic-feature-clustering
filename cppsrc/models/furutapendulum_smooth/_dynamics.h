#pragma once

#include "definition_header.h"
#define USE_FULL_MODEL true

template<class Type, class Array, class constArray>
void dynamics(Array f, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np, const unsigned int mode)
{
    const Type theta = state[0];
    const Type alpha = state[1];
    const Type dtheta = state[2];
    const Type dalpha = state[3];
    
    const Type I1yy = constants[0];
    const Type I2xx = constants[1];
    const Type I2yy = constants[2];
    const Type I2zz = constants[3];
    const Type L1 = constants[4]; 
    const Type l1l1m1 = constants[5]; // = l1*l1*m1, l1: length from base coordinate system to center of mass of link 1 (l2 is in (0, L1)), in some formulations, l1=L1/2, in this case this parameter is only determined by m1
    const Type l2 = constants[6]; // length from the unactuated joint to the center of mass of link 2 (l2 is in (0, L2))
    const Type m2 = constants[7];
    const Type b1 = constants[8];
    const Type b2 = constants[9];
    const Type km = constants[10];
    const Type Rm = constants[11];
        
    const double g = 9.81;
    
    const Type sin_t2 = sin(alpha);
    const Type cos_t2 = cos(alpha);
    const Type sin_2t2 = sin(2.0*alpha);
    
    f[0] = dtheta;
    f[1] = dalpha;
    
    Type M11, M12, u1, M21, M22, u2, C1, C2;
    /*
     * M = 
     *    [ M11, M12;
     *      M21, M22 ]
     * 
     */
    Type denom;
          
    /**
        * Replace in Matlab code:
        * 
        * (regexpr):
        * " (\W?)(\w+)\^2" --> " \1\2*\2"
        * "(\W)(\w+)\^2" --> "\1\2*\2"
        * 
        * (normal):
        * "/2" --> "*0.5"
        * "cos(alpha)" --> "cos_t2"
        * "cos_t2*cos_t2" --> "cos2_t2"
        * "sin(alpha)" --> "sin_t2"
        * 
        */
    
    const double gz = -g;
    const Type sin2_t2 = sin_t2*sin_t2;
    const Type cos2_t2 = cos_t2*cos_t2;
    
    
    Type torque; // control[0] oder state[4] 
    if(mode == 0){
        torque = km*(control[0] - km*dtheta)/Rm;
    }
    else if(mode == 1){
        torque = km*(state[4] - km*dtheta)/Rm;
    }
    
    /////////// l1 and l2 are constants ///////////
    
    M11 = I1yy + I2yy + L1*L1*m2 + l1l1m1 + l2*l2*m2 + I2xx*cos2_t2 - I2yy*cos2_t2 - l2*l2*m2*cos2_t2;
    
    M12 = L1*l2*m2*cos_t2;
    M21 = M12;
    
    M22 = m2*l2*l2 + I2zz;
    
    C1 = I2yy*dalpha*dtheta*sin(2.0*alpha) - I2xx*dalpha*dtheta*sin(2.0*alpha) - L1*dalpha*dalpha*l2*m2*sin_t2 + dalpha*dtheta*l2*l2*m2*sin(2.0*alpha);
    C2 = (I2xx*dtheta*dtheta*sin(2.0*alpha))*0.5 - (I2yy*dtheta*dtheta*sin(2.0*alpha))*0.5 - gz*l2*m2*sin_t2 - (dtheta*dtheta*l2*l2*m2*sin(2.0*alpha))*0.5;

    
      
        /////////// l1=L1/2 and l2=L2/2 are constants ///////////
 /*       
        A1 = I1yy + I2yy + l1l1m1 + L1*L1*m2 + (L2*L2*m2)*0.25 + I2xx*cos2_t2 - I2yy*cos2_t2 - (L2*L2*m2*cos2_t2)*0.25;
        
        B1 = (L1*L2*m2*cos_t2)*0.5;        
        A2 = B1;
        
        B2 = (m2*L2*L2)*0.25 + I2zz;
 
        C1 = -(dalpha*(- dtheta*m2*sin(2*alpha)*L2*L2 + 2*L1*dalpha*m2*sin_t2*L2 + 4*I2xx*dtheta*sin(2*alpha) - 4*I2yy*dtheta*sin(2*alpha)))*0.25;
        
        C2 = (I2xx*dtheta*dtheta*sin(2*alpha))*0.5 - (I2yy*dtheta*dtheta*sin(2*alpha))*0.5 - (L2*gz*m2*sin_t2)*0.5 - (L2*L2*dtheta*dtheta*m2*sin(2*alpha))/8;
*/
        
/*
    /////////// simplified model with less inertia terms ///////////
    const Type J1 = I1yy;
    const Type J2 = I2xx;
    
    M11 = L1*L1*m2 + (L2*L2*m2)*0.25 - (L2*L2*m2*cos_t2*cos_t2)*0.25 + J1;
    M12 = (L1*L2*m2*cos_t2)*0.5;
    M21 = M12;
    M22 = (m2*L2*L2)*0.25 + J2;
    C1 = (L2*L2*dalpha*dtheta*m2*sin(2*alpha))*0.25 - (L1*L2*dalpha*dalpha*m2*sin(alpha))*0.5;
    C2 = (L2*g*m2*sin(alpha))*0.5 - (L2*L2*dtheta*dtheta*m2*sin(2*alpha))*0.125 ;
 */
    
/*
// my old crooked model (DO NOT USE!)

    M11 = I1yy + I2zz + L1*L1*m2 + l1l1m1 + l2*l2*m2 - l2*l2*m2*cos2_t2;
    
    M12 = L1*l2*m2*cos_t2;
    M21 = M12;
    
    M22 = m2*l2*l2 + I2xx*(1.0 + cos2_t2) + I2yy*(1.0 - cos2_t2);
            
    C1 = -I2yy*dalpha*dalpha*sin(2.0*theta) + I2xx*dalpha*dalpha*sin(2.0*theta) - L1*dalpha*dalpha*l2*m2*sin_t2 + dalpha*dtheta*l2*l2*m2*sin(2.0*alpha);
    C2 = -(I2xx*dtheta*dalpha*sin(2.0*theta))*0.5 + (I2yy*dtheta*dalpha*sin(2.0*theta))*0.5 - gz*l2*m2*sin_t2 - (dtheta*dtheta*l2*l2*m2*sin(2.0*alpha))*0.5;

*/
    
    u1 = -C1 - b1*dtheta + torque;
    u2 = -C2 - b2*dalpha;
    
    denom = (M11*M22 - M21*M12); // det([M11 M12; M21 M22])
    f[2] = ( M22*u1 - M12*u2 ) / denom;
    f[3] = ( -M21*u1 + M11*u2 ) / denom;
    if(mode == 0){
        f[4] = 0.0;
    }
    else if(mode == 1) {
        f[4] = control[0];
    }
    
    
    if(nx == 7) {
        const Type f_eta = constants[23];
        const Type f_kd = constants[24];
        const Type f_a0 = constants[25];
        const Type f_aS = constants[26];
        
        const Type w = state[5];
        const Type dw = state[6];
        
        const Type a = f_a0 + f_aS*theta*theta;
        
        f[0] -= f_eta*w;        
        f[5] = dw + a*dtheta;
        f[6] = -f_kd*w - sqrt(4*f_kd)*dw;
    }
}

