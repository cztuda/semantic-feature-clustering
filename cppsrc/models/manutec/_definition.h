#pragma once
#include "definition_header.h"

#define NAME manutec

#define DIM_X 6
#define DIM_U 3
#define DIM_C 4
#define DIM_P 0
#define DIM_INEQC 0
#define DIM_EQC 0
#define DIM_IBC 0



struct DEF {
    
    std::string state_name   [DIM_X] = { "q_1"  , "q_2" , "q_3" , "dq_1", "dq_2", "dq_3" };
    double state_bound_upper [DIM_X] = { 2.97   , 2.01  , 2.86  , 3.00  , 1.5   , 5.2    };
    double state_bound_lower [DIM_X] = { -2.97  , -2.01 , -2.86 , -3.00 , -1.5  , -5.2   };
    double state_fixed_at_t0 [DIM_X] = { true   , true  , true  , true  , true  , true   };
    double state_value_at_t0 [DIM_X] = { 0.0    , -1.5  , 0.0   , 0.0   , 0.0   , 0.0    };
    double state_fixed_at_tf [DIM_X] = { true   , true  , true  , true  , true  , true   };
    double state_value_at_tf [DIM_X] = { 1.0    , -1.95 , 1.0   , 0.0   , 0.0   , 0.0    };
    double state_is_rotation [DIM_X] = { false  , false , false , false , false , false  };
    
    
    std::string control_name   [DIM_U] = { "tau_1"  , "tau_2"   , "tau_3"   };
    double control_bound_upper [DIM_U] = { 7.5      , 7.5       , 7.5       };
    double control_bound_lower [DIM_U] = { -7.5     , -7.5      , -7.5      };
    double control_fixed_at_t0 [DIM_U] = { false    , false     , false     };
    double control_value_at_t0 [DIM_U] = { 0.0      , 0.0       , 0.0       };
    double control_fixed_at_tf [DIM_U] = { false    , false     , false     };
    double control_value_at_tf [DIM_U] = { 0.0      , 0.0       , 0.0       };
    
    
    double tf = 10.0;
    double tf_bound_upper = 100.0;
    double tf_bound_lower = 0.001;
    
    
    Constant constant_list [DIM_C] = {
        {"load_mass", 0.0},
        {"mu_tf"    , 1.0},
        {"mu_energy", 1e-3},
        {"mu_power" , 0.0},
    };
    
    Constant parameter_list [DIM_P] = {};
};


#include "definition_functions.h"
