#pragma once

namespace manutec {
#define ML              ml
#define AQ(i)           aq[i-1]
#define AQD(i)          aqd[i-1]
#define AQDD(i)         aqdd[i-1]
#define FGEN(i)         fgen[i-1]
#define WORKV3(i, j)    workv3[(i-1)*13+(j-1)]
#define MHGES           mhges
#define CMHGES(i)       cmhges[(i-1)]
#define IHHGES(i,j)     ihhges[(i-1)*3+(j-1)]
#define AFOR(i,j)       afor[(i-1)*3+(j-1)]
#define ATOR(i,j)       ator[(i-1)*3+(j-1)]
#define INERTC(i,j,k)   inertc[(i-1)*3*3+(j-1)*3+(k-1)]
#define INERTH(i,j,k)   inerth[(i-1)*3*3+(j-1)*3+(k-1)]
#define RHICM(i,j)      rhicm[(i-1)*3+(j-1)]
#define WABS(i,j)       wabs[(i-1)*3+(j-1)]
#define WORKAM(i,j,k)   workam[(i-1)*6*4+(j-1)*4+(k-1)]
#define WORKAS(i,j)     workas[(i-1)*4+(j-1)]
#define GENVD(i,j)      genvd[(i-1)*3+(j-1)]
#define BETA(i,j)       beta[(i-1)*3+(j-1)]
#define INERTG(i,j,k)   inertg[(i-1)*6*3+(j-1)*3+(k-1)]
#define MREL12(i,j,k)   mrel12[(i-1)*1*3+(j-1)*3+(k-1)]
#define MREL22(i,j,k)   mrel22[(i-1)*1*3+(j-1)*3+(k-1)]
#define RHREL2(i,j)     rhrel2[(i-1)*3+(j-1)]
#define ZETA(i,j)       zeta[(i-1)*3+(j-1)]
#define KW(i,j)         kw[(i-1)*3+(j-1)]
#define WORKM3(i,j,k)   workm3[(i-1)*3*1+(j-1)*1+(k-1)]
#define WORKM6(i,j,k)   workm6[(i-1)*6*5+(j-1)*5+(k-1)]
#define WORKS1(i,j,k)   works1[(i-1)*1*2+(j-1)*2+(k-1)]
#define WORKS2(i,j)     works2[(i-1)*6+(j-1)]
#define WORKV3(i,j)     workv3[(i-1)*13+(j-1)]
#define WORKV6(i,j)     workv6[(i-1)*4+(j-1)]
#define SINP(i,j)       sinp[(i-1)*3+(j-1)]
#define COSP(i,j)       cosp[(i-1)*3+(j-1)]
#define WWORKM(i,j,k)   wworkm[(i-1)*3*6+(j-1)*6+(k-1)]
#define WWORKV(i,j)     wworkv[(i-1)*3+(j-1)]
#define SIN(arg)        sin(arg)
#define COS(arg)        cos(arg)


template<class Type, class Array, class constArray>
void r3m2si(Array aqdd, const Type ml, constArray aq, constArray aqd, constArray fgen){
//
// Simulation model equations of the Manutec r3 robot (DFVLR model 2)
//
// Procedure purpose:
//   This subroutine calculates the generalized accelerations (AQDD) for
//   the Manutec r3 robot of the DFVLR model 2. This model is based on
//   the following assumptions:
//
//   - The last 3 joints (joints 4,5,6) of the robot hand do not move.
//     This corresponds to the case that the brakes in these joints are
//     blocked (joints 4,5,6 are taken into account in model 1).
//
//   - The robot consists of a base body (the environment), three arms
//     and three rotors. The rotors represent the inertia effects of the
//     motors and of the wheels of the gear boxes. The rotors are
//     embedded in the preceeding arms, e.g. the rotor 2, which drives
//     arm2, is embedded in arm1.
//
//   - Arms and rotors are considered to be rigid bodies.
//
//   - Elasticity, backlash and friction in the gear boxes are neglected.
//
//   - The motors are modelled as ideal voltage-torque converters
//     without any dynamic effects. As input arguments of the subroutine
//     the torques at the gear output (FGEN in <Nm>) must be given. If the
//     voltage (U in <V>) at the input of the current regulator of the
//     motor is given, FGEN must be calculated (before calling this
//     subroutine) in the following way:
//         FGEN(1) = -126.0*U(1)
//         FGEN(2) =  252.0*U(2)
//         FGEN(3) =   72.0*U(3)
//
//   - At the robot's tip a load mass (ML) is attached. It can range
//     between 0 ... 15 kg.
//
//   Given the actual value ML of the load mass, the joint angles AQ(i),
//   the derivatives of the joint angles AQD(i), and the driving torques
//   in the joints FGEN(i), the second derivatives of the joint angles
//   AQDD(i) are calculated by this subroutine.
//
// Usage:
//   CALL  R3M2SI (ML, AQ, AQD, FGEN, AQDD)
//
//   ML    :  IN, ARRAY, <kg>, 0<=ML<=15
//            Load mass.
//   AQ    :  IN, ARRAY(3), <rad>, -2.97 <= AQ(1) <= 2.97  (+/- 170 deg),
//                                  -2.01 <= AQ(2) <= 2.01  (+/- 115 deg),
//                                  -2.86 <= AQ(3) <= 2.86  (+/- 164 deg)
//            (The given limits are hardware constraints by limit switches).
//            Vector of generalized coordinates (relative angles between
//            two contigous robot arms).
//            AQ(1) is not used in this subroutine, because it is not
//            needed to calculate AQDD.
//   AQD   :  IN, ARRAY(3), <rad/sec>,
//                                -3.0 <= AQD(1) <= 3.0  (+/- 172 deg/sec),
//                                -1.5 <= AQD(2) <= 1.5  (+/-  86 deg/sec),
//                                -5.2 <= AQD(3) <= 5.2  (+/- 298 deg/sec)
//            Derivative of AQ.
//   FGEN  :  IN, ARRAY(3), <Nm>,     -945.0 <= FGEN(1) <=  945.0,
//                                    -1890.0 <= FGEN(2) <= 1890.0,
//                                     -540.0 <= FGEN(3) <=  540.0
//            Torque at the gear output.
//   AQDD  :  OUT, ARRAY(3), <rad/sec**2>
//            Second derivative of AQ.
//
// Bibliography:
//   A detailed description of the model, together with the body-data
//   (mass, center of mass etc. of the arms and rotors) is given in
//     Otter M., Tuerk S., Mathematical Model of the Manutec r3 Robot,
//            (DFVLR Model No. 2), DFVLR - Oberpfaffenhofen, Institut fuer
//            Dynamik der Flugsysteme, D-8031 Wessling, West Germany,
//            corrected version april 1988.
//
//   This subroutine was generated by the program MYROBOT. See
//     Otter M., Schlegel S., Symbolic generation of efficient simulation
//            codes for robots. 2nd European Simulation Multiconference,
//            June 1-3, 1988, Nice.
//
//   The underlying multibody algorithm is a modified version of
//     Brandl H., Johanni R., Otter M., A very efficient algorithm for the
//            simulation of robots and similar multibody systems without
//            inversion of the mass matrix. IFAC/IFIP/IMACS International
//            Symposium on Theory of Robots, december 3-5, 1986, Vienna.
//
// Remarks:
//   - The limits given for the input variables are not checked in
//     this subroutine. The user is responsible for proper data.
//
//
// Copyright:
//   1988  DFVLR - Institut fuer Dynamik der Flugsysteme
//
// Life cycle:
//   1988 APR  M. Otter, S. Tuerk (DFVLR)             : specified.
//   1988 APR  M. Otter (DFVLR)                       : generated.
//   1988 APR  M. Otter, C. Schlegel, S. Tuerk (DFVLR): tested.
//
//
// published with permission of Prof. M. Otter (granted in 1997 to Prof. O. von Stryk)
// Reference: M. Otter and S. Türk, "The DFVLR models 1 and 2 of the Manutec r3 robot", DFVLR-Mitt. 88-13, Institut für Dynamik der Flugsysteme, Oberpfaffenhofen, Germany, Tech. Rep. 1988
//
// The original Fortran code has been translated to C++ by C. Zelch (2023)
//
//
//-----------------------------------------------------------------------

    
    
    Type mhges;
    Type cmhges[3];
    Type ihhges[3*3];
    Type afor[3*3];
    Type ator[3*3];
    Type inertc[3*3*3];
    Type inerth[3*3*3];
    Type rhicm[3*3];
    Type wabs[3*3];
    Type workam[6*6*4];
    Type workas[10*4];
    Type genvd[1*3];
    Type beta[6*3];
    Type inertg[6*6*3];
    Type mrel12[6*1*3];
    Type mrel22[1*1*3];
    Type rhrel2[1*3];
    Type zeta[6*3];
    Type kw[3*3];
    Type workm3[3*3*1];
    Type workm6[6*6*5];
    Type works1[6*1*2];
    Type works2[1*6];
    Type workv3[3*13];
    Type workv6[6*4];
    Type sinp[1*3];
    Type cosp[1*3];
    Type wworkm[3*3*6];
    Type wworkv[3*3];
    
    //Ccccccccccccccccccccccccc Procedural section cccccccccccccccccccccccccc
    
    WORKV3(3,5) = + ML*9.800000e-01;
    WORKV3(3,6) = + 1.218060e+01 + WORKV3(3,5);
    MHGES = + 6.030000e+01 + ML;
    CMHGES(1) = + 1.688400e+00/MHGES;
    CMHGES(3) = + WORKV3(3,6)/MHGES;
    WORKV3(3,7) = + WORKV3(3,5)*9.800000e-01;
    IHHGES(1,1) = + 7.87048120e+00 + WORKV3(3,7);
    IHHGES(2,2) = + 8.10775640e+00 + WORKV3(3,7);
    SINP(1,2) = + SIN(AQ(2));
    COSP(1,2) = + COS(AQ(2));
    WWORKV(1,1) = + MHGES*CMHGES(1);
    WWORKV(3,1) = + MHGES*CMHGES(3);
    WWORKV(1,2) = + CMHGES(1)*WWORKV(1,1);
    WWORKV(3,2) = + CMHGES(3)*WWORKV(3,1);
    WWORKV(1,3) = + WWORKV(1,2) + WWORKV(3,2);
    WWORKM(1,1,1) = + WWORKV(1,3) - WWORKV(1,2);
    WWORKM(3,3,1) = + WWORKV(1,3) - WWORKV(3,2);
    WWORKM(1,3,1) = - WWORKV(1,1)*CMHGES(3);
    INERTC(1,1,3) = + IHHGES(1,1) - WWORKM(1,1,1);
    INERTC(2,2,3) = + IHHGES(2,2) - WWORKV(1,3);
    INERTC(1,3,3) = - 1.1056800e-02 - WWORKM(1,3,1);
    INERTC(3,3,3) = + 4.3727520e-01 - WWORKM(3,3,1);
    SINP(1,3) = + SIN(AQ(3));
    COSP(1,3) = + COS(AQ(3));
    
    
/// The equations of motion have been generated with the algorithm
/// of Brandl/Johanni/Otter (variant 6)


/// Forward recursion --------------------------------------------------

//  -- Quantities of body 1

//  -- Quantities of body 2
      WABS(2,2) = + SINP(1,2)*AQD(1);
      WABS(3,2) = + COSP(1,2)*AQD(1);
      WORKV3(2,1) = + AQD(1)*AQD(2);
      ZETA(2,2) = + COSP(1,2)*WORKV3(2,1);
      ZETA(3,2) = - SINP(1,2)*WORKV3(2,1);
      KW(1,2) = + 4.9544125e+00*AQD(2) - 2.45219e+00*WABS(3,2);
      KW(2,2) = + 6.7759085e+00*WABS(2,2);
      KW(3,2) = - 2.45219e+00*AQD(2) + 2.311496e+00*WABS(3,2);

// -- Quantities of body 3
      WORKV3(2,7) = - SINP(1,3)*CMHGES(3);
      WORKV3(3,7) = + COSP(1,3)*CMHGES(3);
      WORKM3(2,2,1) = + COSP(1,3)*INERTC(2,2,3);
      WORKM3(3,2,1) = + SINP(1,3)*INERTC(2,2,3);
      WORKM3(2,3,1) = - SINP(1,3)*INERTC(3,3,3);
      WORKM3(3,3,1) = + COSP(1,3)*INERTC(3,3,3);
      INERTC(1,2,3) = - INERTC(1,3,3)*SINP(1,3);
      INERTC(2,2,3) = + WORKM3(2,2,1)*COSP(1,3) - WORKM3(2,3,1)*SINP(1,3);
      INERTC(1,3,3) = + INERTC(1,3,3)*COSP(1,3);
      INERTC(2,3,3) = + WORKM3(2,2,1)*SINP(1,3) + WORKM3(2,3,1)*COSP(1,3);
      INERTC(3,3,3) = + WORKM3(3,2,1)*SINP(1,3) + WORKM3(3,3,1)*COSP(1,3);
      WORKV3(1,8) = + MHGES*CMHGES(1);
      WORKV3(2,8) = + MHGES*WORKV3(2,7);
      WORKV3(3,8) = + MHGES*WORKV3(3,7);
      WORKV3(1,9) = + CMHGES(1)*WORKV3(1,8);
      WORKV3(2,9) = + WORKV3(2,7)*WORKV3(2,8);
      WORKV3(3,9) = + WORKV3(3,7)*WORKV3(3,8);
      WORKV3(1,10) = + WORKV3(1,9) + WORKV3(2,9) + WORKV3(3,9);
      INERTH(1,1,3) = + WORKV3(1,10) - WORKV3(1,9);
      INERTH(2,2,3) = + WORKV3(1,10) - WORKV3(2,9);
      INERTH(3,3,3) = + WORKV3(1,10) - WORKV3(3,9);
      INERTH(1,2,3) = - WORKV3(1,8)*WORKV3(2,7);
      INERTH(1,3,3) = - WORKV3(1,8)*WORKV3(3,7);
      INERTH(2,3,3) = - WORKV3(2,8)*WORKV3(3,7);
      INERTH(1,1,3) = + INERTH(1,1,3) + INERTC(1,1,3);
      INERTH(1,2,3) = + INERTH(1,2,3) + INERTC(1,2,3);
      INERTH(2,2,3) = + INERTH(2,2,3) + INERTC(2,2,3);
      INERTH(1,3,3) = + INERTH(1,3,3) + INERTC(1,3,3);
      INERTH(2,3,3) = + INERTH(2,3,3) + INERTC(2,3,3);
      INERTH(3,3,3) = + INERTH(3,3,3) + INERTC(3,3,3);
      WABS(1,3) = + AQD(3) + AQD(2);
      WORKV3(2,1) = + WABS(3,2)*AQD(3);
      WORKV3(3,1) = - WABS(2,2)*AQD(3);
      WORKV3(1,3) = + WABS(2,2)*5.0e-01;
      WORKV3(2,3) = - AQD(2)*5.0e-01;
      WORKV3(1,4) = - WABS(3,2)*WORKV3(2,3);
      WORKV3(2,4) = + WABS(3,2)*WORKV3(1,3);
      WORKV3(3,4) = + AQD(2)*WORKV3(2,3) - WABS(2,2)*WORKV3(1,3);
      RHICM(1,3) = + MHGES*CMHGES(1);
      RHICM(2,3) = + MHGES*WORKV3(2,7);
      RHICM(3,3) = + MHGES*WORKV3(3,7);
      KW(1,3) = + INERTH(1,1,3)*WABS(1,3) + INERTH(1,2,3)*WABS(2,2) + INERTH(1,3,3)*WABS(3,2);
      KW(2,3) = + INERTH(1,2,3)*WABS(1,3) + INERTH(2,2,3)*WABS(2,2) + INERTH(2,3,3)*WABS(3,2);
      KW(3,3) = + INERTH(1,3,3)*WABS(1,3) + INERTH(2,3,3)*WABS(2,2) + INERTH(3,3,3)*WABS(3,2);


      ATOR(1,2) = - WABS(2,2)*KW(3,2) + WABS(3,2)*KW(2,2);
      ATOR(2,2) = - WABS(3,2)*KW(1,2) + AQD(2)*KW(3,2);
      ATOR(3,2) = - AQD(2)*KW(2,2) + WABS(2,2)*KW(1,2);
      ATOR(1,3) = - WABS(2,2)*KW(3,3) + WABS(3,2)*KW(2,3);
      ATOR(2,3) = - WABS(3,2)*KW(1,3) + WABS(1,3)*KW(3,3);
      ATOR(3,3) = - WABS(1,3)*KW(2,3) + WABS(2,2)*KW(1,3);
      WORKV3(1,6) = + WABS(2,2)*RHICM(3,3) - WABS(3,2)*RHICM(2,3);
      WORKV3(2,6) = + WABS(3,2)*RHICM(1,3) - WABS(1,3)*RHICM(3,3);
      WORKV3(3,6) = + WABS(1,3)*RHICM(2,3) - WABS(2,2)*RHICM(1,3);
      AFOR(1,3) = - WABS(2,2)*WORKV3(3,6) + WABS(3,2)*WORKV3(2,6);
      AFOR(2,3) = - WABS(3,2)*WORKV3(1,6) + WABS(1,3)*WORKV3(3,6);


/// Backward recursion -------------------------------------------------

//  -- Quantities of body 3
      MREL22(1,1,3) = + 4.680e+00 + INERTH(1,1,3);
      WORKV6(1,1) = + ATOR(1,3) - INERTH(1,2,3)*WORKV3(2,1) - INERTH(1,3,3)*WORKV3(3,1) + RHICM(3,3)*WORKV3(2,4) - RHICM(2,3)*WORKV3(3,4);
      WORKV6(2,1) = + ATOR(2,3) - INERTH(2,2,3)*WORKV3(2,1) - INERTH(2,3,3)*WORKV3(3,1) - RHICM(3,3)*WORKV3(1,4) + RHICM(1,3)*WORKV3(3,4);
      WORKV6(3,1) = + ATOR(3,3) - INERTH(2,3,3)*WORKV3(2,1) - INERTH(3,3,3)*WORKV3(3,1) + RHICM(2,3)*WORKV3(1,4) - RHICM(1,3)*WORKV3(2,4);
      WORKV6(4,1) = + AFOR(1,3) - RHICM(3,3)*WORKV3(2,1) + RHICM(2,3)*WORKV3(3,1) - MHGES*WORKV3(1,4);
      WORKV6(5,1) = + AFOR(2,3) - RHICM(1,3)*WORKV3(3,1) - MHGES*WORKV3(2,4);
      RHREL2(1,3) = + FGEN(3) + WORKV6(1,1);
      MREL12(1,1,3) = + 7.80e-02 + INERTH(1,1,3) + 5.0e-01*RHICM(3,3);
      WORKV6(1,4) = + WORKV6(1,1) - 5.0e-01*WORKV6(5,1);
      WORKV6(2,4) = + WORKV6(2,1) + 5.0e-01*WORKV6(4,1);
      WORKM6(1,1,4) = + INERTH(1,1,3) - RHICM(3,3)*(-5.0e-01);
      WORKM6(5,1,4) = - RHICM(3,3) + MHGES*(-5.0e-01);
      WORKM6(2,2,4) = + INERTH(2,2,3) + RHICM(3,3)*5.0e-01;
      WORKM6(4,2,4) = + RHICM(3,3) + MHGES*5.0e-01;
      INERTG(1,1,2) = + 4.95441250e+00 + WORKM6(1,1,4) - 5.0e-01*WORKM6(5,1,4);
      INERTG(2,2,2) = + 6.77590850e+00 + WORKM6(2,2,4) + 5.0e-01*WORKM6(4,2,4);
      INERTG(1,3,2) = - 2.452190e+00 + INERTH(1,3,3) - 5.0e-01*RHICM(1,3);
      INERTG(2,3,2) = + INERTH(2,3,3) - 5.0e-01*RHICM(2,3);
      INERTG(3,3,2) = + 2.3114960e+00 + INERTH(3,3,3);
      INERTG(1,5,2) = - 1.158250e+01 - RHICM(3,3) - 5.0e-01*MHGES;
      INERTG(3,5,2) = + 9.7180e+00 + RHICM(1,3);
      INERTG(2,6,2) = - 9.7180e+00 - RHICM(1,3);
      BETA(1,2) = + ATOR(1,2) + WORKV6(1,4);
      BETA(2,2) = + ATOR(2,2) + WORKV6(2,4);
      BETA(3,2) = + ATOR(3,2) + WORKV6(3,1);
      WORKS2(1,1) = + MREL12(1,1,3)/MREL22(1,1,3);
      WORKS2(1,2) = + INERTH(1,2,3)/MREL22(1,1,3);
      WORKS2(1,3) = + INERTH(1,3,3)/MREL22(1,1,3);
      WORKS2(1,5) = - RHICM(3,3)/MREL22(1,1,3);
      WORKS2(1,6) = + RHICM(2,3)/MREL22(1,1,3);
      INERTG(1,1,2) = + INERTG(1,1,2) - MREL12(1,1,3)*WORKS2(1,1);
      INERTG(1,2,2) = + INERTH(1,2,3) - MREL12(1,1,3)*WORKS2(1,2);
      INERTG(2,2,2) = + INERTG(2,2,2) - INERTH(1,2,3)*WORKS2(1,2);
      INERTG(1,3,2) = + INERTG(1,3,2) - MREL12(1,1,3)*WORKS2(1,3);
      INERTG(2,3,2) = + INERTG(2,3,2) - INERTH(1,2,3)*WORKS2(1,3);
      INERTG(3,3,2) = + INERTG(3,3,2) - INERTH(1,3,3)*WORKS2(1,3);
      INERTG(1,5,2) = + INERTG(1,5,2) - MREL12(1,1,3)*WORKS2(1,5);
      INERTG(2,5,2) = - INERTH(1,2,3)*WORKS2(1,5);
      INERTG(3,5,2) = + INERTG(3,5,2) - INERTH(1,3,3)*WORKS2(1,5);
      INERTG(1,6,2) = + RHICM(2,3) - MREL12(1,1,3)*WORKS2(1,6);
      INERTG(2,6,2) = + INERTG(2,6,2) - INERTH(1,2,3)*WORKS2(1,6);
      INERTG(3,6,2) = - INERTH(1,3,3)*WORKS2(1,6);
      BETA(1,2) = + BETA(1,2) - WORKS2(1,1)*RHREL2(1,3);
      BETA(2,2) = + BETA(2,2) - WORKS2(1,2)*RHREL2(1,3);
      BETA(3,2) = + BETA(3,2) - WORKS2(1,3)*RHREL2(1,3);

//  -- Quantities of body 2
      MREL22(1,1,2) = + 5.7330e+01 + INERTG(1,1,2);
      WORKV6(1,1) = + BETA(1,2) - INERTG(1,2,2)*ZETA(2,2) - INERTG(1,3,2)*ZETA(3,2);
      WORKV6(2,1) = + BETA(2,2) - INERTG(2,2,2)*ZETA(2,2) - INERTG(2,3,2)*ZETA(3,2);
      WORKV6(3,1) = + BETA(3,2) - INERTG(2,3,2)*ZETA(2,2) - INERTG(3,3,2)*ZETA(3,2);
      RHREL2(1,2) = + FGEN(2) + WORKV6(1,1);
      WORKS1(3,1,2) = + SINP(1,2)*INERTG(1,2,2) + COSP(1,2)*INERTG(1,3,2);
      WORKS1(6,1,2) = + SINP(1,2)*INERTG(1,5,2) + COSP(1,2)*INERTG(1,6,2);
      WORKV6(3,3) = + SINP(1,2)*WORKV6(2,1) + COSP(1,2)*WORKV6(3,1);
      WORKAS(1,4) = + COSP(1,2) + SINP(1,2);
      WORKAS(2,4) = + COSP(1,2) - SINP(1,2);
      WORKAS(4,4) = + WORKAS(1,4)*WORKAS(2,4);
      WORKAS(3,4) = + COSP(1,2)*SINP(1,2);
      WORKAS(5,4) = + WORKAS(3,4) + WORKAS(3,4);
      WORKAS(1,1) = + INERTG(3,3,2) + INERTG(2,2,2);
      WORKAS(2,1) = + INERTG(3,3,2) - INERTG(2,2,2);
      WORKAS(5,1) = + WORKAS(1,1)/2.0e+00;
      WORKAS(6,1) = + WORKAS(2,1)/2.0e+00;
      WORKAS(9,1) = + WORKAS(4,4)*WORKAS(6,1) + WORKAS(5,4)*INERTG(2,3,2);
      WORKAM(2,2,4) = + WORKAS(5,1) + WORKAS(9,1);
      WORKAS(1,3) = + INERTG(3,6,2) + INERTG(2,5,2);
      WORKAS(2,3) = + INERTG(3,6,2) - INERTG(2,5,2);
      WORKAS(5,3) = + WORKAS(1,3)/2.0e+00;
      WORKAS(6,3) = + WORKAS(2,3)/2.0e+00;
      WORKAS(4,3) = - INERTG(3,5,2) - INERTG(2,6,2);
      WORKAS(8,3) = + WORKAS(4,3)/2.0e+00;
      WORKAS(9,3) = + WORKAS(4,4)*WORKAS(6,3) - WORKAS(5,4)*WORKAS(8,3);
      WORKAM(2,5,4) = + WORKAS(5,3) + WORKAS(9,3);
      INERTG(3,3,1) = + 1.160e+00 + WORKAM(2,2,4);
      WORKS2(1,3) = + WORKS1(3,1,2)/MREL22(1,1,2);
      WORKS2(1,6) = + WORKS1(6,1,2)/MREL22(1,1,2);
      INERTG(3,3,1) = + INERTG(3,3,1) - WORKS1(3,1,2)*WORKS2(1,3);
      INERTG(3,6,1) = + WORKAM(2,5,4) - WORKS1(3,1,2)*WORKS2(1,6);
      BETA(3,1) = + WORKV6(3,3) - WORKS2(1,3)*RHREL2(1,2);

//  -- Quantities of body 1
      MREL22(1,1,1) = + 1.433250e+01 + INERTG(3,3,1);
      WORKV6(3,1) = + BETA(3,1) - INERTG(3,6,1)*9.810e+00;
      RHREL2(1,1) = + FGEN(1) + WORKV6(3,1);


/// Forward recursion --------------------------------------------------

//  -- Quantities of body 1
      GENVD(1,1) = + RHREL2(1,1)/MREL22(1,1,1);

//  -- Quantities of body 2
      RHREL2(1,2) = + RHREL2(1,2) - WORKS1(3,1,2)*GENVD(1,1) - WORKS1(6,1,2)*9.810e+00;
      GENVD(1,2) = + RHREL2(1,2)/MREL22(1,1,2);
      WORKV6(2,2) = + ZETA(2,2) + SINP(1,2)*GENVD(1,1);
      WORKV6(3,2) = + ZETA(3,2) + COSP(1,2)*GENVD(1,1);
      WORKV6(5,2) = + SINP(1,2)*9.810e+00;
      WORKV6(6,2) = + COSP(1,2)*9.810e+00;

//  -- Quantities of body 3
      RHREL2(1,3) = + RHREL2(1,3) - MREL12(1,1,3)*GENVD(1,2) - INERTH(1,2,3)*WORKV6(2,2) - INERTH(1,3,3)*WORKV6(3,2) + RHICM(3,3)*WORKV6(5,2) - RHICM(2,3)*WORKV6(6,2);
      GENVD(1,3) = + RHREL2(1,3)/MREL22(1,1,3);


      AQDD(1) = + GENVD(1,1);
      AQDD(2) = + GENVD(1,2);
      AQDD(3) = + GENVD(1,3);
}


template<class Type, class Array, class constArray>
void dynamics(Array f, constArray x, constArray u, constArray p, constArray c){
//**********************************************************************
// Purpose:
// -------
//    Computation of the right hand side F of the differential equations
//
//                  DX/DT  =  F(X, U, P)
//
//    in phase number IPHASE.
//
// ENTRY-Parameters:
// ----------------
//    X........the state variables at time T (array of length 6)
//    U........the control variables at time T (array of length 3)
//    P........the control parameters (array of length 1)
//
// EXIT-Parameters:
// ----------------
//    F........computed value of the right hand side of the
//             differential equations in phase IPHASE at time T
//             (array of length 6)
//**********************************************************************
//
    const Type fac1 = Type(-126.0);
    const Type fac2 = Type(252.0);
    const Type fac3 = Type(72.0);
    
    const Type ml = (const Type)c[0];
    
    f[0] = x[3];
    f[1] = x[4];
    f[2] = x[5];
    
    Type aq[3];
    Type aqd[3];
    Type aqdd[3];
    Type fgen[] = {fac1*u[0], fac2*u[1], fac3*u[2]};
    
    aq[0] = x[0];
    aq[1] = x[1];
    aq[2] = x[2];
    aqd[0] = x[3];
    aqd[1] = x[4];
    aqd[2] = x[5];
    
    r3m2si(&aqdd[0], ml, &aq[0], &aqd[0], &fgen[0]);
    
    f[3] = aqdd[0];
    f[4] = aqdd[1];
    f[5] = aqdd[2];
    
//     f[6] = u[0]*u[0] + u[1]*u[1] + u[2]*u[2]; 
}
}

template<class Type, class Array, class constArray>
void dynamics(Array f, constArray state, constArray control, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np, const unsigned int mode)
{
    manutec::dynamics<Type, Array, constArray>(f, state, control, parameter, constants);
}