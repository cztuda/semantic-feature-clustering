#pragma once
#include "definition_header.h"

#define NAME furutapendulum_smooth

#define DIM_X 7
#define DIM_U 1
#define DIM_C 27
#define DIM_P 0
#define DIM_INEQC 0
#define DIM_EQC 0
#define DIM_IBC 0
#define DIM_DYNMODES 2



struct DEF {
    
    // state and control properties:
    std::string state_name   [DIM_X] = { "arm pos"   , "pnd pos" , "arm velo", "pnd velo", "int_du"  , "w"       , "dw"      };
    double state_bound_upper [DIM_X] = { 115.0/180*PI,  2*PI     ,  60.0     ,  60.0     ,  5.0      , 1e10+1    , 1e10+1    };
    double state_bound_lower [DIM_X] = {-115.0/180*PI, -2*PI     , -60.0     , -60.0     , -5.0      , -1e10+1   , -1e10+1   };
    bool   state_fixed_at_t0 [DIM_X] = { true        , true      , true      , true      , true      , true      , true      };
    double state_value_at_t0 [DIM_X] = { 0.0         , 0.0       , 0.0       , 0.0       , 0.0       , 0.0       , 0.0       };
    bool   state_fixed_at_tf [DIM_X] = { true        , true      , true      , true      , false     , false     , false     };
    double state_value_at_tf [DIM_X] = { 0.0         , PI        , 0.0       , 0.0       , 0.0       , 0.0       , 0.0       };
    bool   state_is_rotation [DIM_X] = { false       , false     , false     , false     , false     , false     , false     };
    
    
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
        {"J1",   5.491/96000.0}, //  0
        {"J2x",  0.399384/12.0}, //  1
        {"J2y",  0.399384/12.0}, //  2
        {"J2z",  0.399384/12.0}, //  3
        {"L1",        0.085},    //  4
        {"l1l1m1",0.095*0.0425*0.0425}, //  5  // m1=0.095, l1=0.0425
        {"l2",        0.0645},   //  6
        {"m2",        0.024},    //  7
        {"b1",        5.0e-6},   //  8
        {"b2",        1.0e-6},   //  9
        {"k_M",       0.042},    // 10
        {"R_M",       8.4},      // 11
        {"Q_11",      0.4},      // 12
        {"Q_22",      0.0},      // 13
        {"Q_33",      0.1},      // 14
        {"Q_44",      0.0},      // 15
        {"R_11",      0.0},      // 16
        {"F_11",      0.0},      // 17
        {"F_22",      0.2},      // 18
        {"F_33",      0.0},      // 19
        {"F_44",      0.2},      // 20
        {"T_1",       0.0},      // 21
        {"alpha",     1.6},      // 22
        {"f_eta",     0.5},      // 23
        {"f_kd",      2.8},      // 24
        {"f_a0",      0.0},      // 25
        {"f_aS",      6.7},      // 26
    };
    
    Constant parameter_list [DIM_P] = {};
    
    std::string dynamic_mode_list[DIM_DYNMODES] = {"standard", "smooth"};
};


#include "definition_functions.h"
