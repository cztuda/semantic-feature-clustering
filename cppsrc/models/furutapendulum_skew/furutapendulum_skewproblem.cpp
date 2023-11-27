
#include <Eigen/Geometry>
#include <cmath>
#include <boost/serialization/export.hpp>
#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>

#include "furutapendulum_skewproblem.h"
#include "_definition.h"

using namespace ofc::models;
using namespace ofc::math;


BOOST_CLASS_EXPORT_IMPLEMENT(Furutapendulum_skewProblem)


extern "C" OptimalControlProblem* create_default() {
    OptimalControlProblem* p = new Furutapendulum_skewProblem();
    p->initialize();    
    return p;
}
extern "C" OptimalControlProblem* create_copy(const OptimalControlProblem* p) {
    const Furutapendulum_skewProblem* pcast = dynamic_cast<const Furutapendulum_skewProblem*>(p);
    if(!pcast) return 0;
    OptimalControlProblem* p2 = new Furutapendulum_skewProblem(*pcast);
    p2->initialize(); 
    return p2;
}
extern "C" OptimalControlProblem* create_unserialize(std::istream s) {
    OptimalControlProblem* p;
    OptimalControlProblem::unserializeOCP(&p, s);
    return p;
}
extern "C" void destroy(OptimalControlProblem* p) {
    delete p;
}


long Furutapendulum_skewProblem::derivativeLibraryInstanceCounter = 0;
std::unique_ptr<CppAD::cg::LinuxDynamicLib<double>> Furutapendulum_skewProblem::derivativeLibrary = 0;


void Furutapendulum_skewProblem::initialize()
{
    DEF def;
    unsigned int nx, nu, nc, np;
    getDimensions(nx, nu, nc, np);
    
    this->setProblemName(TO_STRING(NAME));
    
    for(int i = 0; i < nx; ++i) {
        this->stateVariableLowerBound[i] = def.state_bound_lower[i];
        this->stateVariableUpperBound[i] = def.state_bound_upper[i];
        this->stateIsFixedAtInitialTime[i] = def.state_fixed_at_t0[i];
        this->stateIsFixedAtFinalTime[i] = def.state_fixed_at_tf[i];
        this->initialState[i] = def.state_value_at_t0[i];
        this->finalState[i] = def.state_value_at_tf[i];
        this->stateIsUnconstrainedAngle[i] = def.state_is_rotation[i];
    }
    
    for(int i = 0; i < nu; ++i) {
        this->controlVariableLowerBound[i] = def.control_bound_lower[i];
        this->controlVariableUpperBound[i] = def.control_bound_upper[i];
        this->controlIsFixedAtInitialTime[i] = def.control_fixed_at_t0[i];
        this->controlIsFixedAtFinalTime[i] = def.control_fixed_at_tf[i];
        this->initialControl[i] = def.control_value_at_t0[i];
        this->finalControl[i] = def.control_value_at_tf[i];
    }
    
    this->setFinalTime(def.tf, def.tf_bound_lower, def.tf_bound_upper);
        
    for(unsigned int i = 0; i < nc; ++i) {
        this->addConstant(def.constant_list[i].value, def.constant_list[i].description);   
    }
    
    for(unsigned int i = 0; i < np; ++i) {
        this->addParameter(def.parameter_list[i].value, def.parameter_list[i].description);   
    }
    
    if(DIM_DYNMODES > 0){
        this->setDynamicModeDescription(0, def.dynamic_mode_list[0]);
    }
    for(unsigned int i = 1; i < DIM_DYNMODES; ++i) {
        this->addDynamicMode(def.dynamic_mode_list[i]);
    }
}

void Furutapendulum_skewProblem::manageDerivativeLibrary()
{
    if(Furutapendulum_skewProblem::derivativeLibraryInstanceCounter > 0)
        Furutapendulum_skewProblem::derivativeLibraryInstanceCounter++;        
    if(Furutapendulum_skewProblem::derivativeLibraryInstanceCounter == 0) 
    {
        if(CppAD::cg::system::isFile("furutapendulum_skewDerivatives.so")) {
            Furutapendulum_skewProblem::derivativeLibrary = std::unique_ptr<CppAD::cg::LinuxDynamicLib<double>>( new CppAD::cg::LinuxDynamicLib<double>("furutapendulum_skewDerivatives.so") );
            Furutapendulum_skewProblem::derivativeLibraryInstanceCounter++;
        } else {
            Furutapendulum_skewProblem::derivativeLibraryInstanceCounter = -1;
        }
    } 
}

std::string Furutapendulum_skewProblem::getThisLibraryPathname() const
{
    Dl_info info;
    
    int res = dladdr((void*)&create_default, &info);
    if(res) {
        return std::string(info.dli_fname);
    }
    else {
        return std::string("");
    }
    
}

Furutapendulum_skewProblem::Furutapendulum_skewProblem() : OptimalControlProblem(DIM_X, DIM_U)
{
    manageDerivativeLibrary();
};

Furutapendulum_skewProblem::Furutapendulum_skewProblem(const Furutapendulum_skewProblem& p) : OptimalControlProblem(p) 
{
    manageDerivativeLibrary();
};

ofc::models::Furutapendulum_skewProblem::~Furutapendulum_skewProblem()
{
    if(this->derivativeLibraryInstanceCounter > 0)
        this->derivativeLibraryInstanceCounter--;
    if(this->derivativeLibraryInstanceCounter == 0)
        Furutapendulum_skewProblem::derivativeLibrary = 0;
}



int ofc::models::Furutapendulum_skewProblem::getNumberOfImplicitNonlinearBoundaryConditions() const
{ return DIM_IBC;}
int ofc::models::Furutapendulum_skewProblem::getNumberOfEqualityConstraints() const
{ return DIM_EQC;}
int ofc::models::Furutapendulum_skewProblem::getNumberOfInequalityConstraints() const
{ return DIM_INEQC;}


bool Furutapendulum_skewProblem::objectiveMayerTerm(double& fobj, double tf, uint nx, uint nu, uint np, const double* x0, const double* u0, const double* p, const double* xf, const double* uf) const {
  
    objective_Mayer<double, double*, const double*>(fobj, tf, x0, u0, xf, uf, this->getConstants(), p, nx, nu, DIM_C, np); 
  return true;
}


bool Furutapendulum_skewProblem::objectiveLagrangeTermIntegrand(double& fobj, uint nx, uint nu, uint np, const double* x, const double* u, const double* p) const {
  
    objective_Lagrange<double, double*, const double*>(fobj, x, u, this->getConstants(), p, nx, nu, DIM_C, np);  
  return true;
}


bool Furutapendulum_skewProblem::evaluate(double* f, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {
    
    dynamics<double, double*, const double*>(f, state, control, this->getConstants(), parameter, nx, nu, DIM_C, np, this->getDynamicMode());
    return true;
}


bool ofc::models::Furutapendulum_skewProblem::NonlinearEqualityConstraints(double* nec, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const
{
    
    constraint_NEC<double, double*, const double*>(nec, state, control, this->getConstants(), parameter, nx,  nu, DIM_C, np);
    return true;
}


bool ofc::models::Furutapendulum_skewProblem::NonlinearInequalityConstraints(double* nec, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const
{
    
    constraint_NIC<double, double*, const double*>(nec, state, control, this->getConstants(), parameter, nx, nu, DIM_C, np);
    return true;
}


bool ofc::models::Furutapendulum_skewProblem::NonlinearBoundaryConditions(double* xf, double* uf, const double* x0, const double* u0, const double* p, double t0, double tf, uint nx, uint nu, uint np) const
{
    return true;
}

bool ofc::models::Furutapendulum_skewProblem::implicitNonlinearBoundaryConditions(double* rb, const double* x0, const double* u0, const double* p, double* xf, double* uf, double t0, double tf, uint nx, uint nu, uint np, uint nrb) const
{
    return true;
}


void retrieveJacobians_v1(double* J_x, double* J_u, double* J_c, double* J_p, std::unique_ptr< CppAD::cg::GenericModel<double>>& model, const double* state, const double* control, const double* constants, const double* parameter, unsigned int nx, unsigned int nu, unsigned int nc, unsigned int np, unsigned int ny)
{
    unsigned int n = nx+nu+nc+np;
    std::vector<double> input(n);
    memcpy(input.data(), state, nx*sizeof(double));
    memcpy(input.data()+nx, control, nu*sizeof(double));
    memcpy(input.data()+nx+nu, constants, nc*sizeof(double));
    memcpy(input.data()+nx+nu+nc, parameter, np*sizeof(double));
    
    
    std::vector<double> jac(ny*n);
    model->Jacobian(input, jac);
    if(J_x){
        for(size_t i = 0; i < ny; ++i){
            // copy row after row, each of the ny rows has nx elements starting from 0
            memcpy(&J_x[nx*i], &jac.data()[n*i], nx*sizeof(double));
        }
    }
    if(J_u){
        for(size_t i = 0; i < ny; ++i){
            // copy row after row, each of the ny rows has nu elements starting from nx
            memcpy(&J_u[nu*i], &jac.data()[n*i + nx], nu*sizeof(double));
        }
    }
    if(J_c){
        for(size_t i = 0; i < ny; ++i){
            // copy row after row, each of the ny rows has nc elements starting from nx+nu
            memcpy(&J_c[nc*i], &jac.data()[n*i + nx + nu], nc*sizeof(double));
        }
    }
    if(J_p){
        for(size_t i = 0; i < ny; ++i){
            // copy row after row, each of the ny rows has np elements starting from nx+nu+nc
            memcpy(&J_p[np*i], &jac.data()[n*i + nx + nu + nc], np*sizeof(double));
        }
    }
}

void retrieveJacobians_v2(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, std::unique_ptr< CppAD::cg::GenericModel<double>>& model, const double* state_t0, const double* control_t0, const double* constants, const double* parameter, const double* state_tf, const double* control_tf, double tf, unsigned int nx, unsigned int nu, unsigned int nc, unsigned int np, unsigned int ny)
{
    unsigned int n = 2*nx + 2*nu + np + nc + 1;
    std::vector<double> input(n);
    memcpy(input.data(), state_t0, nx*sizeof(double));
    memcpy(input.data()+nx, control_t0, nu*sizeof(double));
    memcpy(input.data()+nx+nu, state_tf, nx*sizeof(double));
    memcpy(input.data()+2*nx+nu, control_tf, nu*sizeof(double));
    memcpy(input.data()+2*nx+2*nu, constants, nc*sizeof(double));
    memcpy(input.data()+2*nx+2*nu+nc, parameter, np*sizeof(double));
    input[nx+nu+np+nc+nx+nu] = tf;
    
    
    std::vector<double> jac(ny*n);
    model->Jacobian(input, jac);
    for(size_t i = 0; i < ny; ++i){
        if(J_x0) {
            // copy row after row, each of the ny rows has nx elements starting from 0
            memcpy(&J_x0[nx*i], &jac.data()[n*i], nx*sizeof(double));
        }
        if(J_u0) {
            // copy row after row, each of the ny rows has nu elements starting from nx
            memcpy(&J_u0[nu*i], &jac.data()[n*i + nx], nu*sizeof(double));
        }
        if(J_xf) {
            // copy row after row, each of the ny rows has nx elements starting from nx+nu
            memcpy(&J_xf[nx*i], &jac.data()[n*i + nx+nu], nx*sizeof(double));
        }
        if(J_uf) {
            // copy row after row, each of the ny rows has nu elements starting from nx+nu+nx
            memcpy(&J_uf[nu*i], &jac.data()[n*i + nx+nu+nx], nu*sizeof(double));
        }
        if(J_c) {
            // copy row after row, each of the ny rows has nc elements starting from 2*nx+2*nu
            memcpy(&J_p[nc*i], &jac.data()[n*i + 2*nx+2*nu], nc*sizeof(double));
        }
        if(J_p) {
            // copy row after row, each of the ny rows has np elements starting from 2*nx+2*nu+nc
            memcpy(&J_p[np*i], &jac.data()[n*i + 2*nx+2*nu+nc], np*sizeof(double));
        }
        if(J_tf) {
            // copy row after row, each of the ny rows has 1 element starting from nx+nu+np+nc+nx+nu
            memcpy(&J_u0[1*i], &jac.data()[n*i + nx+nu+np+nc+nx+nu], 1*sizeof(double));
        }
    }
}


bool ofc::models::Furutapendulum_skewProblem::dynamicsJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, ofc::models::uint nx, ofc::models::uint nu, ofc::models::uint np) const
{
    if(this->derivativeLibraryInstanceCounter < 0) return false;
    
    auto model = this->derivativeLibrary->model("dynamics");
    retrieveJacobians_v1(J_x, J_u, J_c, J_p, model, state, control, this->getConstants(), parameter, nx, nu, this->getNumberOfConstants(), np, nx);
    return true;
}


bool ofc::models::Furutapendulum_skewProblem::objectiveLagrangeJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, ofc::models::uint nx, ofc::models::uint nu, ofc::models::uint np) const
{
    if(this->derivativeLibraryInstanceCounter < 0) return false;
    
    auto model = this->derivativeLibrary->model("Lagrange");
    retrieveJacobians_v1(J_x, J_u, J_c, J_p, model, state, control, this->getConstants(), parameter, nx, nu, this->getNumberOfConstants(), np, 1);
    return true;
}

// (double& fobj, double tf, uint nx, uint nu, uint np, const double* x0, const double* u0, const double* p, const double* xf, const double* uf)
bool ofc::models::Furutapendulum_skewProblem::objectiveMayerJacobians(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, const double* state_t0, const double* control_t0, const double* parameter, const double tf, const double* state_tf, const double* control_tf, ofc::models::uint nx, ofc::models::uint nu, ofc::models::uint np) const
{
    auto model = this->derivativeLibrary->model("Mayer");
    retrieveJacobians_v2(J_x0, J_u0, J_c, J_p, J_xf, J_uf, J_tf, model, state_t0, control_t0, this->getConstants(), parameter, state_tf, control_tf, tf, nx, nu, this->getNumberOfConstants(), np, 1);
    return true;
}


bool ofc::models::Furutapendulum_skewProblem::NECJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, ofc::models::uint nx, ofc::models::uint nu, ofc::models::uint np) const
{
    if(this->getNumberOfEqualityConstraints() > 0){
        auto model = this->derivativeLibrary->model("NEC");
        retrieveJacobians_v1(J_x, J_u, J_c, J_p, model, state, control, this->getConstants(), parameter, nx, nu, this->getNumberOfConstants(), np, this->getNumberOfEqualityConstraints());
        return true;
    }
    else return false;
}


bool ofc::models::Furutapendulum_skewProblem::NICJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, ofc::models::uint nx, ofc::models::uint nu, ofc::models::uint np) const
{
    if(this->getNumberOfInequalityConstraints() > 0){
        auto model = this->derivativeLibrary->model("NIC");
        retrieveJacobians_v1(J_x, J_u, J_c, J_p, model, state, control, this->getConstants(), parameter, nx, nu, this->getNumberOfConstants(), np, this->getNumberOfInequalityConstraints());
        return true;
    }
    else return false;
}

bool ofc::models::Furutapendulum_skewProblem::NBCJacobians(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, ofc::models::uint nx, ofc::models::uint nu, ofc::models::uint np, const double* x0, const double* u0, const double* p, double t0, double tf, double* xf, double* uf) const
{
    if(this->getNumberOfImplicitNonlinearBoundaryConditions() > 0) {
        auto model = this->derivativeLibrary->model("NBC");
        retrieveJacobians_v2(J_x0, J_u0, J_c, J_p, J_xf, J_uf, J_tf, model, x0, u0, this->getConstants(), p, xf, uf, tf, nx, nu, this->getNumberOfConstants(), np, this->getNumberOfImplicitNonlinearBoundaryConditions());
        return true;
    }
    else return false;
}
