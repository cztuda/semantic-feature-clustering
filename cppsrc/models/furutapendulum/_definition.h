#pragma once
#include "definition_header.h"

#define NAME furutaPendulum

#define DIM_X 4
#define DIM_U 1
#define DIM_C 25
#define DIM_P 0
#define DIM_INEQC 0
#define DIM_EQC 0
#define DIM_IBC 0



struct DEF {
    
    std::string state_name   [DIM_X] = { "pnd pos" , "arm pos" , "pnd velo", "arm velo" };
    double state_bound_upper [DIM_X] = { 2*PI      , 2*PI      , 100.0     , 100.0      };
    double state_bound_lower [DIM_X] = { -2*PI     , -2*PI     , -100.0    , -100.0     };
    bool   state_fixed_at_t0 [DIM_X] = { true      , true      , true      , true       };
    double state_value_at_t0 [DIM_X] = { 0.0       , 0.0       , 0.0       , 0.0        };
    bool   state_fixed_at_tf [DIM_X] = { true      , false     , true      , true       };
    double state_value_at_tf [DIM_X] = { 0.0       , PI        , 0.0       , 0.0        };
    bool   state_is_rotation [DIM_X] = { false     , false     , false     , false      };
    
    
    std::string control_name   [DIM_U] = { "torque"    };
    double control_bound_upper [DIM_U] = { 30.0         };
    double control_bound_lower [DIM_U] = { -30.0        };
    bool   control_fixed_at_t0 [DIM_U] = { false       };
    double control_value_at_t0 [DIM_U] = { 0.0         };
    bool   control_fixed_at_tf [DIM_U] = { false       };
    double control_value_at_tf [DIM_U] = { 0.0         };
    
    
    double tf = 1.0;
    double tf_bound_upper = 1.0;
    double tf_bound_lower = 1.0;
    
    // parameters for a pendulum described in Cazzolato and Prime, 2011
    Constant constant_list [DIM_C] = {
        {"J1xx",      0.0},      //  0
        {"J1yy",      0.0},      //  1
        {"J1zz",      2.48e-2},  //  2
        {"J2xx",      0.0},      //  3
        {"J2yy",      0.0},      //  4
        {"J2zz",      3.86e-3},  //  5
        {"m1",        0.3},      //  6
        {"m2",        0.075},    //  7
        {"L1",        0.278},    //  8
        {"L2",        0.3},      //  9
        {"l1",        0.15},     // 10
        {"l2",        0.148},    // 11
        {"b1",        1e-4},     // 12
        {"b2",        2.8e-4},   // 13
        {"Q_11",      0.0},      // 14
        {"Q_22",      0.0},      // 15
        {"Q_33",      0.0},      // 16
        {"Q_44",      0.0},      // 17
        {"R_11",      0.0},      // 18
        {"F_11",      0.0},      // 19
        {"F_22",      0.0},      // 20
        {"F_33",      0.0},      // 21
        {"F_44",      1.0},      // 22
        {"T_1",       0.0},      // 23
        {"alpha",     1.6},      // 24
    };
    
    // parameters for the Quanser pendulum
//     Constant constant_list [DIM_C] = {
//         {"J1xx",      0.0},      //  0
//         {"J1yy",      0.0012},   //  1
//         {"J1zz",      0.0012},   //  2
//         {"J2xx",      0.0},      //  3
//         {"J2yy",      0.000998}, //  4
//         {"J2zz",      0.000998}, //  5
//         {"m1",        0.257},    //  6
//         {"m2",        0.127},    //  7
//         {"L1",        0.216},    //  8
//         {"L2",        0.337},    //  9
//         {"l1",        0.156},    // 10
//         {"l2",        0.0619},   // 11
//         {"b1",        0.0024},   // 12
//         {"b2",        0.0001},   // 13
//         {"Q_11",      1.0},      // 14
//         {"Q_22",      1.0},      // 15
//         {"Q_33",      1.0},      // 16
//         {"Q_44",      1.0},      // 17
//         {"R_11",      1.0},      // 18
//         {"F_11",      1.0},      // 19
//         {"F_22",      1.0},      // 20
//         {"F_33",      1.0},      // 21
//         {"F_44",      1.0},      // 22
//         {"T_1",       1.0},      // 23
//     };
    
    Constant parameter_list [DIM_P] = {};
};


#include "definition_functions.h"
