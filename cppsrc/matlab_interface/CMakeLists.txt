
SET(MATLAB_SRC
    MFun.h
    MFun.cpp
    SingleVariableMFun.h
    SingleVariableMFun.cpp
)


matlab_add_mex(NAME problem_interface_mex SRC ${MATLAB_SRC} problem_interface_mex.cpp R2018a)
target_include_directories(problem_interface_mex SYSTEM PUBLIC ${Matlab_INCLUDE_DIR})

matlab_add_mex(NAME universalFunction_mex SRC ${MATLAB_SRC} universalFunction_mex.cpp R2018a)
target_include_directories(universalFunction_mex SYSTEM PUBLIC ${Matlab_INCLUDE_DIR})

#add_subdirectory(cpp_interface)

