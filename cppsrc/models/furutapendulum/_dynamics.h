#pragma once

#include <cmath>


template<class Type, class Array, class constArray>
void dynamics(Array f, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np, const unsigned int mode)
{
    const Type theta1 = state[0];
    const Type theta2 = state[1];
    const Type dtheta1 = state[2];
    const Type dtheta2 = state[3];
    
//     const Type J1xx = constants[0];
//     const Type J1yy = constants[1];
    const Type J1zz = constants[2];
    const Type J2xx = constants[3];
    const Type J2yy = constants[4];
    const Type J2zz = constants[5];
    const Type m1 = constants[6];
    const Type m2 = constants[7];
    const Type L1 = constants[8];
    const Type L2 = constants[9];
    const Type l1 = constants[10];
    const Type l2 = constants[11];
    const Type b1 = constants[12];
    const Type b2 = constants[13];
    
    const double g = 9.81;
    
    const Type sin_t2 = sin(theta2);
    const Type cos_t2 = cos(theta2);
    
    // Benjamin Seth Cazzolato and Zebb Prime, 2011
    // with equation (19) solved for the accelerations
//     const Type A1 = J1zz  +  m1*l1*l1  +  m2*L1*L1  +  (J2yy + m2*l2*l2)*sin_t2*sin_t2  +  J2xx*cos_t2*cos_t2;
//     const Type B1 = m2*L1*l2*cos_t2;
//     const Type u1 = control[0]  -  ( -m2*L1*l2*sin_t2*dtheta2*dtheta2  +  dtheta1*dtheta2*sin(2*theta2)*(m2*l2*l2 + J2yy - J2xx)  +  b1*dtheta1 );
//     
//     const Type A2 = m2*L1*l2*cos_t2;
//     const Type B2 = m2*l2*l2  +  J2zz;
//     const Type u2 = 0  -  ( 0.5*dtheta1*dtheta1*sin(2*theta2)*(-m2*l2*l2 - J2yy + J2xx)  +  b2*dtheta2  +  g*m2*l2*sin_t2 );
    
    
    // Quanser (Rotary Pendulum - Workbook - Student)
//     auto A1 = m2*L1*L1  +  0.25*m2*L2*L2  -  0.25*m2*L2*L2*cos_t2*cos_t2  +  J1zz;
//     auto B1 = -0.5*m2*L2*L1*cos_t2;
//     auto u1 = control[0]  -  ( b1*dtheta1  -  (0.5*m2*L2*L2*sin_t2*cos_t2)*dtheta1*dtheta2  -  (0.5*m2*L2*L1*sin_t2)*dtheta2*dtheta2 );
//     
//     auto A2 = -0.5*m2*L2*L1*cos_t2;
//     auto B2 = J2zz + 0.25*m2*L2*L2;
//     auto u2 = -b2*dtheta2  +  0.25*m2*L2*L2*cos_t2*sin_t2*dtheta1*dtheta1  +  0.5*m2*L2*g*sin_t2;
//     
//     const Type det = (A1*B2 - A2*B1);
    
    f[0] = state[2];
    f[1] = state[3];
//     f[2] = ( B2*u1 - B1*u2 ) / det;
//     f[3] = ( -A2*u1 + A1*u2 ) / det;
    
    // Benjamin Seth Cazzolato and Zebb Prime, 2011
    // with simplified equation (34)
    auto J1 = J1zz + m1*l1*l1;
    auto J2 = J2zz + m2*l2*l2;
    auto J0 = J1 + m2*L1*L1;
    
    auto denom = J0*J2 + J2*J2*sin_t2*sin_t2 - m2*m2*L1*L1*l2*l2*cos_t2*cos_t2;    
    f[2] = (
        - J2*b1 * dtheta1
        + m2*L1*l2*cos_t2*b2 * dtheta2 
        - J2*J2*sin(2*theta2) * dtheta1*dtheta2
        - 0.5*J2*m2*L1*l2*cos_t2*sin(2*theta2) * dtheta1*dtheta1
        + J2*m2*L1*l2*sin_t2
        
        + J2*control[0]
        + (1/2)*m2*m2*l2*l2*L1*sin(2*theta2)*g
    ) / denom;
    f[3] = (
        + m2*L1*l2*cos_t2*b1 * dtheta1
        - b2*(J0+J2*sin_t2*sin_t2) * dtheta2
        + m2*L1*l2*J2*cos_t2*sin(2*theta2) * dtheta1*dtheta2
        - 0.5*sin(2*theta2)*(J0*J2+J2*J2*sin_t2*sin_t2) * dtheta1*dtheta1
        - 0.5*m2*m2*L1*L1*l2*l2*sin(2*theta2) * dtheta2*dtheta2
        
        - m2*L1*l2*cos_t2 * control[0]
        - m2*l2*sin_t2*(J0+J2*sin_t2*sin_t2)*g
    ) / denom;
    
    
    
    
    
//     auto a1 = b2*m2*cos_t2*l2*l1*dtheta2  -  b1*m2*l2*l2*dtheta1  +  m2*l2*l2*(m2*l2*l2*dtheta1 + J2zz*dtheta1)*sin(2*theta2)*dtheta2  +  ( (m2*m2*l2*l2*l2*l1 + m2*l2*l1*J2zz)*dtheta2*dtheta2 - m2*m2*l2*l2*l2*l1*cos_t2*cos_t2*dtheta2*dtheta2 - m2*m2*l2*l2*l1*g*cos_t2 )*sin_t2  -  b1*J2zz*dtheta1  +  (m2*l2*l2 + J2zz)*control[0];
//     auto denom = (l1*l1*J2zz + l2*l2*J1zz)*m2  +  (m2*m2*l2*l2*l2*l2 + m2*l2*l2*J2zz)*sin_t2*sin_t2  -  m2*m2*l2*l2*l1*l1*cos_t2*cos_t2  +  m2*m2*l2*l2*l1*l1  +  J2zz*J1zz;
    
    
}
