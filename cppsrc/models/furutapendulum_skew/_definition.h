#pragma once
#include "definition_header.h"

#define NAME furutapendulum_skew

#define DIM_X 4
#define DIM_U 1
#define DIM_C 27
#define DIM_P 0
#define DIM_INEQC 0
#define DIM_EQC 0
#define DIM_IBC 0
#define DIM_DYNMODES 0



struct DEF {
    
    // state and control properties:
    std::string state_name   [DIM_X] = { "arm pos"   , "pnd pos" , "arm velo", "pnd velo"};
    double state_bound_upper [DIM_X] = { 115.0/180*PI,  2*PI     ,  60.0     ,  60.0     };
    double state_bound_lower [DIM_X] = {-115.0/180*PI, -2*PI     , -60.0     , -60.0     };
    bool   state_fixed_at_t0 [DIM_X] = { true        , true      , true      , true      };
    double state_value_at_t0 [DIM_X] = { 0.0         , 0.0       , 0.0       , 0.0       };
    bool   state_fixed_at_tf [DIM_X] = { true        , true      , true      , true      };
    double state_value_at_tf [DIM_X] = { 0.0         , PI        , 0.0       , 0.0       };
    bool   state_is_rotation [DIM_X] = { false       , false     , false     , false     };
    
    
    std::string control_name   [DIM_U] = { "voltage"};
    double control_bound_upper [DIM_U] = { 5.0          };
    double control_bound_lower [DIM_U] = { -5.0         };
    bool   control_fixed_at_t0 [DIM_U] = { false         };
    double control_value_at_t0 [DIM_U] = { 0.0           };
    bool   control_fixed_at_tf [DIM_U] = { false         };
    double control_value_at_tf [DIM_U] = { 0.0           };
    
    // general problem properties:
    double tf = 3.0;
    double tf_bound_upper = 4.0;
    double tf_bound_lower = 0.1;
    
    
    Constant constant_list [DIM_C] = {
        {"Q_11",      0.4},      //  0
        {"Q_22",      0.0},      //  1
        {"Q_33",      0.1},      //  2
        {"Q_44",      0.0},      //  3
        {"R_11",      0.0},      //  4
        {"F_11",      0.0},      //  5
        {"F_22",      0.2},      //  6
        {"F_33",      0.0},      //  7
        {"F_44",      0.2},      //  8
        {"T_1",       0.0},      //  9
        {"alpha",     1.6},      // 10
        {"g_rotY0",   0.0},      // 11
        {"g_rotZ0",   0.0},      // 12
        {"g_norm",    9.81},     // 13
        {"J1",   5.491/96000.0}, // 14
        {"J2x",  0.399384/12.0}, // 15
        {"J2y",  0.399384/12.0}, // 16
        {"J2z",  0.399384/12.0}, // 17
        {"L1",        0.085},    // 18
        {"l1",        0.0425},   // 19
        {"l2",        0.0645},   // 20
        {"m1",        0.095},    // 21
        {"m2",        0.024},    // 22
        {"k_M",       0.042},    // 23
        {"R_M",       8.4},      // 24
        {"fr1_sigm0", 0.0},      // 25
        {"fr2_sigm0", 0.0},      // 26
    };
    
    Constant parameter_list [DIM_P] = {};
    
    std::string dynamic_mode_list[DIM_DYNMODES] = {};
};


#include "definition_functions.h"
