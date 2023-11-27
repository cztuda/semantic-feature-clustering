#pragma once

/**
 * Selection of friction model based on the "Review and comparison of dry friction force models" (Pennestri et al., Nonlinear Dynamics 2016)
 * - LuGre, Elasto-plastic and Gonthier model can model "stiction", "static force" and "pre-sliding" -effects
 * - Gonthier has a low dependency on the ODE solver (i.e., does not need a stiff solver)
 * - Gonthier requires two additional state variables (plus derivatives) and 8 parameters 
 * - LuGre requires one additional state variable (plus derivative) and 7 parameters
 * 
 */


template<class Type, class Array, class constArray>
void dynamics(Array f, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np, const unsigned int mode)
{
    const Type theta = state[0];
    const Type alpha = state[1];
    const Type dtheta = state[2];
    const Type dalpha = state[3];
//     const Type w1 = state[4];
//     const Type dw1 = state[5];
//     const Type w2 = state[6];
//     const Type dw2 = state[7];
    
    const Type g_rotY0 = constants[11];
    const Type g_rotZ0 = constants[12];
    const Type g_norm = constants[13];
    const Type I1yy = constants[14];
    const Type I2xx = constants[15];
    const Type I2yy = constants[16];
    const Type I2zz = constants[17];
    const Type L1 = constants[18];
    const Type l1 = constants[19];
    const Type l2 = constants[20]; // length from the unactuated joint to the center of mass of link 2 (l2 is in (0, L2))
    const Type m1 = constants[21];
    const Type m2 = constants[22];
    const Type km = constants[23];
    const Type Rm = constants[24];
            
    const Type sin_t2 = sin(alpha);
    const Type cos_t2 = cos(alpha);
    const Type sin_2t2 = sin(2.0*alpha);
        
    Type M11, M12, u1, M21, M22, u2, C1, C2, G11, G12, G13, G21, G22, G23;
    /*
     * M = 
     *    [ M11, M12;
     *      M21, M22 ]
     * 
     */
    Type denom;
          
    const Type sin2_t2 = sin_t2*sin_t2;
    const Type cos2_t2 = cos_t2*cos_t2;
    
    
    const Type torque = km*(control[0] - km*dtheta)/Rm;
    
    // compute gravity vector:
    const Type gx = sin(g_rotY0)*cos(g_rotZ0)*g_norm;
    const Type gy = sin(g_rotY0)*sin(g_rotZ0)*g_norm;
    const Type gz = cos(g_rotY0)*g_norm;
    
    
    // compute parts of the dynamics equation
    M11 = I1yy + I2yy +  L1*L1*m2 +  l1*l1*m1 +  l2*l2*m2 + I2xx*cos2_t2 - I2yy*cos2_t2 -  l2*l2*m2*cos2_t2;
    
    M12 = L1*l2*m2*cos_t2;
    M21 = M12;
    
    M22 = I2zz +  l2*l2*m2;
    
    C1 = I2yy*dalpha*dtheta*sin_2t2 - I2xx*dalpha*dtheta*sin_2t2 - L1*dalpha*dalpha*l2*m2*sin_t2 + dalpha*dtheta*l2*l2*m2*sin_2t2;
    C2 = -(dtheta*dtheta*sin_2t2*(I2yy - I2xx +  l2*l2*m2))*0.5;
    
    G11 = - L1*m2*sin(theta) - l1*m1*sin(theta) - l2*m2*sin_t2*cos(theta);
    G12 = l1*m1*cos(theta) + L1*m2*cos(theta) - l2*m2*sin_t2*sin(theta);
    G13 = 0;
    G21 = -l2*m2*cos_t2*sin(theta);
    G22 = l2*m2*cos_t2*cos(theta);
    G23 = l2*m2*sin_t2;
    
    const Type G1 = G11*gx + G12*gy + G13*gz;
    const Type G2 = G21*gx + G22*gy + G23*gz;
    
    Type friction1, friction2;
    const Type zero = Type(0.0);
    

    // simple damping
    friction1 = - constants[25]*dtheta;
    friction2 = - constants[26]*dalpha;

    
    
    
    // formulate the dynamics equation
    u1 = -C1 - G1 + friction1 + torque;
    u2 = -C2 - G2 + friction2;
    
    denom = (M11*M22 - M21*M12); // det([M11 M12; M21 M22])
        
    
    f[0] = dtheta;
    f[1] = dalpha;
    f[2] = ( M22*u1 - M12*u2 ) / denom;
    f[3] = ( -M21*u1 + M11*u2 ) / denom;
//     f[4] = dw1;
    // f[5] computed in friction model
//     f[6] = dw2;
    // f[7] computed in friction model
    
}

