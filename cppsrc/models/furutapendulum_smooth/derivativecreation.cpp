

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <cppad/cg.hpp>

#include "furutapendulum_smoothproblem.h"
#include "_definition.h"

using namespace std;
using namespace CppAD::cg;
using namespace CppAD;

const std::string LIBNAME = "furutapendulum_smoothDerivatives";

typedef CG<double> CGD;
typedef CppAD::AD<CGD> ADCG;
typedef CppAD::vector<ADCG> vec;
typedef unsigned int uint;


inline void assignToXIn(vec& v, const vec& xIn, const size_t n, const size_t offset){
    for(size_t i = 0; i < n; ++i){
        v[i] = xIn[i + offset];
    }
}



int main() {
    
    ofc::models::Furutapendulum_smoothProblem* p = new ofc::models::Furutapendulum_smoothProblem();
    p->initialize();
    
    
    size_t nx = p->getNumberOfStateVariables();
    size_t nu = p->getNumberOfControlVariables();
    size_t nc = p->getNumberOfConstants();
    size_t np = p->getNumberOfParameters();
    size_t n = nx + nu + nc + np;
    
    vec xIn(n);
    for(size_t i = 0; i < n; ++i){
        xIn[i] = 1.0;
    }    
    
    // ***** create derivatives for the dynamics *****
    Independent(xIn);
    vec yOut(nx);
    
    vec states(nx);
    assignToXIn(states, xIn, nx, 0);
    
    vec controls(nu);
    assignToXIn(controls, xIn, nu, nx);
    
    
    vec constants(nc);
    const double* problemconstants = p->getConstants();
    for(size_t i = 0; i < nc; ++i){
        constants[i] = problemconstants[i];
    }
    assignToXIn(constants, xIn, nc, nx+nu);
        
    vec parameters(np);
    assignToXIn(parameters, xIn, np, nx+nu+nc);
    
    
    dynamics<ADCG, vec&, const vec&>(yOut, states, controls, constants, parameters, nx, nu, nc, np, 0);
    ADFun<CGD> fun_dynamics(xIn, yOut);
    ModelCSourceGen<double> cgen_dynamics(fun_dynamics, "dynamics");
    cgen_dynamics.setCreateJacobian(true);
    cgen_dynamics.setCreateForwardOne(true);
    cgen_dynamics.setCreateReverseOne(true);
    cgen_dynamics.setCreateReverseTwo(true);
    
    ModelLibraryCSourceGen<double> libcgen(cgen_dynamics);
   
    // ***** create derivatives for the Lagrange term *****
    Independent(xIn);
    assignToXIn(states, xIn, nx, 0);
    assignToXIn(controls, xIn, nu, nx);
    assignToXIn(constants, xIn, nc, nx+nu);
    assignToXIn(parameters, xIn, np, nx+nu+nc);
    
    yOut.resize(1);
    ADCG yOutScalar;
    yOut[0] = yOutScalar;
    
    objective_Lagrange<ADCG, vec&, const vec&>(yOutScalar, states, controls, constants, parameters, nx, nu, nc, np);
    ADFun<CGD> fun_Lagrange(xIn, yOut);
    ModelCSourceGen<double> cgen_Lagrange(fun_Lagrange, "Lagrange");
    cgen_Lagrange.setCreateJacobian(true);
    cgen_Lagrange.setCreateForwardOne(true);
    cgen_Lagrange.setCreateReverseOne(true);
    cgen_Lagrange.setCreateReverseTwo(true);
    
    libcgen.addModel(cgen_Lagrange);
    
    // ***** create derivatives for the Nonlinear Equality Constraints *****
    if(p->getNumberOfEqualityConstraints() > 0){
        Independent(xIn);
        assignToXIn(states, xIn, nx, 0);
        assignToXIn(controls, xIn, nu, nx);
        assignToXIn(constants, xIn, nc, nx+nu);
        assignToXIn(parameters, xIn, np, nx+nu+nc);
        
        yOut.resize(p->getNumberOfEqualityConstraints());
        
        constraint_NEC<ADCG, vec&, const vec&>(yOut, states, controls, constants, parameters, nx, nu, nc, np);
        ADFun<CGD> fun_NEC(xIn, yOut);
        ModelCSourceGen<double> cgen_NEC(fun_NEC, "NEC");
        cgen_NEC.setCreateJacobian(true);
        cgen_NEC.setCreateForwardOne(true);
        cgen_NEC.setCreateReverseOne(true);
        cgen_NEC.setCreateReverseTwo(true);
    
        libcgen.addModel(cgen_NEC);
    }
    
    // ***** create derivatives for the Nonlinear Inequality Constraints *****
    if(p->getNumberOfInequalityConstraints() > 0){
        Independent(xIn);
        assignToXIn(states, xIn, nx, 0);
        assignToXIn(controls, xIn, nu, nx);
        assignToXIn(constants, xIn, nc, nx+nu);
        assignToXIn(parameters, xIn, np, nx+nu+nc);
        
        yOut.resize(p->getNumberOfInequalityConstraints());
        
        constraint_NIC<ADCG, vec&, const vec&>(yOut, states, controls, constants, parameters, nx, nu, nc, np);
        ADFun<CGD> fun_NIC(xIn, yOut);
        ModelCSourceGen<double> cgen_NIC(fun_NIC, "NIC");
        cgen_NIC.setCreateJacobian(true);
        cgen_NIC.setCreateForwardOne(true);
        cgen_NIC.setCreateReverseOne(true);
        cgen_NIC.setCreateReverseTwo(true);
        
        libcgen.addModel(cgen_NIC);
    }
    
    // ***** create derivatives for the Mayer term *****
    xIn.resize(n+nx+nu);
    Independent(xIn);
    assignToXIn(states, xIn, nx, 0);
    assignToXIn(controls, xIn, nu, nx);
    
    vec states_tf(nx);
    assignToXIn(states_tf, xIn, nx, nx+nu);
    
    vec controls_tf(nu);
    assignToXIn(controls_tf, xIn, nu, nx+nu+nx);
    
    assignToXIn(constants, xIn, nc, 2*nx+2*nu);
    assignToXIn(parameters, xIn, np, 2*nx+2*nu+nc);
    
    yOut.resize(1);
    yOut[0] = yOutScalar;
    const double tf = p->getFinalTime();
    objective_Mayer<ADCG, vec&, const vec&>(yOutScalar, tf, states, controls, states_tf, controls_tf, constants, parameters, nx, nu, nc, np);
    ADFun<CGD> fun_Mayer(xIn, yOut);
    ModelCSourceGen<double> cgen_Mayer(fun_Mayer, "Mayer");
    cgen_Mayer.setCreateJacobian(true);
    cgen_Mayer.setCreateForwardOne(true);
    cgen_Mayer.setCreateReverseOne(true);
    cgen_Mayer.setCreateReverseTwo(true);
    
    libcgen.addModel(cgen_Mayer);
    
    // ***** create derivatives for the Nonlinear Boundary Conditions *****
//     if(p->getNumberOfNonlinearBoundaryConditions() > 0){
//     Independent(xIn);
//     assignToXIn(states, xIn, nx, 0);
//     assignToXIn(controls, xIn, nu, nx);
//     assignToXIn(parameters, xIn, np, nx+nu);
//     
//     yOut.resize(1);
//     yOut[0] = yOutScalar;
//     
//     // constraint_NBC(int i, Type t0, Type tf, constArray state_t0, constArray control_t0, constArray state_tf, constArray control_tf, constArray constants, constArray parameter, const unsigned int nx, const unsigned int nu, const unsigned int nc, const unsigned int np);
//     constraint_NBC<ADCG, vec&, const vec&>(yOutScalar, states, controls, constants, parameters, nx, nu, nc, np);
//     ADFun<CGD> fun_NBC(xIn, yOut);
//     ModelCSourceGen<double> cgen_NBC(fun_NBC, "NBC");
//     cgen_NBC.setCreateJacobian(true);
//     cgen_NBC.setCreateForwardOne(true);
//     cgen_NBC.setCreateReverseOne(true);
//     cgen_NBC.setCreateReverseTwo(true);
    
//         libcgen.addModel(cgen_NBC);
//     }
    

    // compile source code
    DynamicModelLibraryProcessor<double> dmlp(libcgen, LIBNAME);

    GccCompiler<double> compiler;
    bool loadLib = false;
    dmlp.createDynamicLibrary(compiler, loadLib);
    cout << "Created library " << LIBNAME << endl;
    
    return 0;
}
