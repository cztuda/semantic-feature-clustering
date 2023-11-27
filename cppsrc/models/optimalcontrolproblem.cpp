
#include <sstream>
#include <iostream>
#include <limits>
#include <dlfcn.h>

#include <boost/archive/polymorphic_text_oarchive.hpp>
#include <boost/archive/polymorphic_text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

#include "optimalcontrolproblem.h"

#include "../math/EigenPlus.h"


using namespace ofc::models;
using namespace ofc::math;
using namespace ofc;

BOOST_CLASS_EXPORT_IMPLEMENT(OptimalControlProblem)


OptimalControlProblem::OptimalControlProblem()
{  
    this->build(2, 1);
}


ofc::models::OptimalControlProblem::OptimalControlProblem(const ofc::models::uint nx, const ofc::models::uint nu)
{
    this->build(nx, nu);
}


OptimalControlProblem::OptimalControlProblem (const OptimalControlProblem& p) :
constants(p.constants),
constantsDescription(p.constantsDescription),
parameters(p.parameters),
parameterDescription(p.parameterDescription),
dynamicMode(p.dynamicMode),
dynamicModeDescription(p.dynamicModeDescription),
externalConstantsArray(0),
problemName(p.problemName),
finalControl(p.finalControl),
finalState(p.finalState),
initialControl(p.initialControl),
initialState(p.initialState),
t0(p.t0),
tf(p.tf),
tfLower(p.tfLower),
tfUpper(p.tfUpper),
stateVariableLowerBound(p.stateVariableLowerBound),
stateVariableUpperBound(p.stateVariableUpperBound),
stateIsFixedAtInitialTime(p.stateIsFixedAtInitialTime),
stateIsFixedAtFinalTime(p.stateIsFixedAtFinalTime),
controlVariableLowerBound(p.controlVariableLowerBound),
controlVariableUpperBound(p.controlVariableUpperBound),
controlIsFixedAtInitialTime(p.controlIsFixedAtInitialTime),
controlIsFixedAtFinalTime(p.controlIsFixedAtFinalTime),
stateIsUnconstrainedAngle(p.stateIsUnconstrainedAngle),
controlIsUnconstrainedAngle(p.controlIsUnconstrainedAngle),
initialGuessIsStateLagrangeOneFunction(p.initialGuessIsStateLagrangeOneFunction)
{
    if(p.customInitialGuessLagrange) customInitialGuessLagrange = p.customInitialGuessLagrange->copy();
    else customInitialGuessLagrange = 0;
    if(p.customInitialGuessState) customInitialGuessState = p.customInitialGuessState->copy();
    else customInitialGuessState = 0;
    if(p.customInitialGuessControl) customInitialGuessControl = p.customInitialGuessControl->copy();
    else customInitialGuessControl = 0;
    
}


OptimalControlProblem* OptimalControlProblem::copy() const {
    OptimalControlProblem* o = new OptimalControlProblem (*this);
    return o;
}


void ofc::models::OptimalControlProblem::serializeOCP(const OptimalControlProblem* ocp, std::ostream& s)
{
    boost::archive::polymorphic_text_oarchive  ar(s);
    ar << ocp;
}


bool ofc::models::OptimalControlProblem::unserializeOCP(ofc::models::OptimalControlProblem** ocp, std::istream& s)
{
    ofc::models::OptimalControlProblem* tmp;
    boost::archive::polymorphic_text_iarchive ia(s);
    // read class state from archive
    ia >> tmp;
    *ocp = tmp;
    return true;
}


std::string ofc::models::OptimalControlProblem::serialize() const
{
    std::ostringstream os;
    OptimalControlProblem::serializeOCP(this, os);
    return os.str();
}


ofc::models::OptimalControlProblem * ofc::models::OptimalControlProblem::unserialize(const std::string& s)
{
    std::istringstream is(s);
    OptimalControlProblem* ptr;
    OptimalControlProblem::unserializeOCP(&ptr, is);
    return ptr;    
}


OptimalControlProblem::~OptimalControlProblem()
{  
    if(this->customInitialGuessLagrange != 0) delete this->customInitialGuessLagrange;
    if(this->customInitialGuessState != 0) delete this->customInitialGuessState;
    if(this->customInitialGuessControl != 0) delete this->customInitialGuessControl;
}


void OptimalControlProblem::build(const uint nx, const uint nu)
{
    customInitialGuessLagrange = 0;
    customInitialGuessState = 0;
    customInitialGuessControl = 0;
    initialGuessIsStateLagrangeOneFunction = false;
    
    // initialize all attributes  
    setProblemName("Generic Optimal Control Problem");
    this->stateVariableUpperBound = TVectorX(nx);
    this->stateVariableLowerBound = TVectorX(nx);
    this->controlVariableUpperBound = TVectorX(nu);
    this->controlVariableLowerBound = TVectorX(nu);
    this->stateIsUnconstrainedAngle = VectorXb::Constant(nx, false);
    this->controlIsUnconstrainedAngle = VectorXb::Constant(nu, false);
    
    this->initialState = TVectorX(nx);
    this->finalState = TVectorX(nx);
    this->initialControl = TVectorX(nu);
    this->finalControl = TVectorX(nu);
    
    this->stateIsFixedAtInitialTime = VectorXb::Constant(nx, false);
    this->stateIsFixedAtFinalTime = VectorXb::Constant(nx, false);
    this->controlIsFixedAtInitialTime = VectorXb::Constant(nu, false);
    this->controlIsFixedAtFinalTime = VectorXb::Constant(nu, false);
    
    this->setInitialTime(0.0);
    this->setFinalTime(1.0, 1.0, 1.0);
    
    constants.clear();
    constantsDescription.clear();
    externalConstantsArray = 0;
    parameters.clear();
    parameterDescription.clear();
    dynamicModeDescription.clear();
    addDynamicMode("<default>");
    this->dynamicMode = 0;
}


void ofc::models::OptimalControlProblem::build()
{    
    int nx = this->getNumberOfStateVariables();
    int nx_old = this->stateIsFixedAtInitialTime.size();
    
    if(nx != nx_old){    
        stateIsFixedAtInitialTime.conservativeResize(nx);
        stateIsFixedAtFinalTime.conservativeResize(nx);
        stateIsUnconstrainedAngle.conservativeResize(nx);
        stateVariableLowerBound.conservativeResize(nx);
        stateVariableUpperBound.conservativeResize(nx);
        
        if(this->customInitialGuessState && (
            (!this->initialGuessIsStateLagrangeOneFunction && this->customInitialGuessState->NY() < nx)
            || (this->initialGuessIsStateLagrangeOneFunction && this->customInitialGuessState->NY() < nx+1)
        )) {
            this->customInitialGuessState = 0; // remove the custom initial control guess if its dimension does not fit any more
        }
        
        if(nx > nx_old) {
            // initialize new data
            for(int i = nx_old; i < nx; ++i) {
                stateIsFixedAtInitialTime[i] = false;
                stateIsFixedAtFinalTime[i] = false;
                stateIsUnconstrainedAngle[i] = false;
                stateVariableLowerBound[i] = 0;
                stateVariableUpperBound[i] = 0;
            }
        }
    }
    
    int nu = this->getNumberOfControlVariables();
    int nu_old = this->stateIsFixedAtInitialTime.size();
    
    if(nu != nu_old){ 
        this->controlVariableUpperBound.conservativeResize(nu);
        this->controlVariableLowerBound.conservativeResize(nu);
        this->initialControl.conservativeResize(nu);
        this->finalControl.conservativeResize(nu);
        this->controlIsFixedAtInitialTime.conservativeResize(nu, false);
        this->controlIsFixedAtFinalTime.conservativeResize(nu, false);
        
        if(this->customInitialGuessControl && this->customInitialGuessControl->NY() < nu) {
            this->customInitialGuessControl = 0; // remove the custom initial control guess if its dimension does not fit any more
        }
        
        if(nu > nu_old) {
            // initialize new data
            for(int i = nu_old; i < nu; ++i) {
                controlIsFixedAtInitialTime[i] = false;
                controlIsFixedAtFinalTime[i] = false;
                controlIsUnconstrainedAngle[i] = false;
                controlVariableLowerBound[i] = 0;
                controlVariableUpperBound[i] = 0;
            }
        }
    }
}


std::string ofc::models::OptimalControlProblem::getThisLibraryPathname() const
{
    Dl_info info;
    
    int res = dladdr((void*)&OptimalControlProblem::serializeOCP, &info);
    if(res) {
        return std::string(info.dli_fname);
    }
    else {
        return std::string("");
    }
    
}


int OptimalControlProblem::getNumberOfStateVariables()  const
{
    return this->initialState.rows();
}


int OptimalControlProblem::getNumberOfControlVariables()  const 
{
    return this->initialControl.rows();
}


int OptimalControlProblem::getNumberOfParameters() const 
{
    return this->parameters.size();
}


int OptimalControlProblem::getNumberOfConstants() const 
{
    return this->constants.size();
}


void defaultInitialGuessLagrange(double& l, double tau)
{
    l = 10.0*tau; // set the linear interpolation 0->10.0 as initial guess for the Lagrange term
}


void defaultInitialGuessState(const OptimalControlProblem* ocp, int nx, double tau, double* x)
{
    TVectorX s0, sf;
    VectorXb fixedAt0, fixedAtf;
    double low, upp;
    ocp->getStateAtInitialTime(s0);
    ocp->getStateAtFinalTime(sf);
    TVectorX state = s0*(1.0-tau) + tau * sf;
    ocp->getStateIsFixedAtInitialTime(fixedAt0);
    ocp->getStateIsFixedAtFinalTime(fixedAtf);
    
    for(int i = 0; i < nx; i++){
        if(!fixedAtf[i]){
            if(fixedAt0[i]){
                x[i] = s0[i]; // set as the initial state
            } else {
                ocp->getControlVariableBounds(low, upp, i);
                x[i] = low + (upp-low)*0.501; // in between upper and lower bound, with a small offset to avoid 0 for symmetric bounds
            }
        }
        else {
            if(fixedAt0[i]){
                x[i] = state[i]; // set at interpolation
            } else {
                x[i] = sf[i]; // set as the final state
            }
        }
    }
}


void defaultInitialGuessControl(const OptimalControlProblem* ocp, int nu, double tau, double* u)
{
    TVectorX c0, cf;
    VectorXb fixedAt0, fixedAtf;
    double low, upp;
    ocp->getControlAtInitialTime(c0);
    ocp->getControlAtFinalTime(cf);
    TVectorX control = c0*(1.0-tau) + tau * cf;
    ocp->getControlIsFixedAtInitialTime(fixedAt0);
    ocp->getControlIsFixedAtFinalTime(fixedAtf);
    
    for(int i = 0; i < nu; i++){
        if(!fixedAtf[i]){
            if(fixedAt0[i]){
                u[i] = c0[i]; // set as the initial control
            } else {
                ocp->getControlVariableBounds(low, upp, i);
                u[i] = low + (upp-low)*0.501; // in between upper and lower bound, with a small offset to avoid 0 for symmetric bounds
            }
        }
        else {
            if(fixedAt0[i]){
                u[i] = control[i]; // set at interpolation
            } else {
                u[i] = cf[i]; // set at final control
            }
        }
    }
}


void ofc::models::OptimalControlProblem::initialEstimate(int nx, int nu, double tau, double* x, double* u, double& l) const
{
    //assume that the output of the custom functions is at least as required (but dimension can be higher)
    
    double scaledTau;
    
    std::vector<double> tmp;
    size_t n;
    
    //// initial guess for the control:
    if(this->customInitialGuessControl) {
        n = this->customInitialGuessControl->NY();
        scaledTau = tau*(this->customInitialGuessControl->getTf()-this->customInitialGuessControl->getT0());
        if(n == nu) {
            this->customInitialGuessControl->evaluate(u, &scaledTau);
        }
        else { // else buffer the output
            tmp.reserve(n);
            this->customInitialGuessControl->evaluate(tmp.data(), &scaledTau);
            memcpy(u, tmp.data(), nu*sizeof(double));
        }
    }
    else {
        defaultInitialGuessControl(this, nu, tau, u);
    }
    
    if(this->initialGuessIsStateLagrangeOneFunction && this->customInitialGuessState){
        //// initial guess for the state AND the Lagrange term
        tmp.reserve(this->customInitialGuessState->NY());
        scaledTau = tau*(this->customInitialGuessState->getTf()-this->customInitialGuessState->getT0());
        this->customInitialGuessState->evaluate(tmp.data(), &scaledTau);
        //    std::cout << "in=" << tau << ", res=" << tmp(0) << ", " << tmp(1) << ", " << tmp(2) << std::endl;
        memcpy(x, tmp.data(), nx*sizeof(double));
        l = tmp[nx];
    }
    else {
        //// initial guess for the Lagrange term:
        if(this->customInitialGuessLagrange) {
            scaledTau = tau*(this->customInitialGuessLagrange->getTf()-this->customInitialGuessLagrange->getT0());
            this->customInitialGuessLagrange->evaluate(&l, &scaledTau);
        }
        else {
            defaultInitialGuessLagrange(l, tau);
        }
        
        //// initial guess for the state:
        if(this->customInitialGuessState) {
            scaledTau = tau*(this->customInitialGuessState->getTf()-this->customInitialGuessState->getT0());
            n = this->customInitialGuessState->NY();
            if(n == nx) {
                this->customInitialGuessState->evaluate(x, &scaledTau);
            }
            else { // else buffer the output
                tmp.reserve(n);
                this->customInitialGuessState->evaluate(tmp.data(), &scaledTau);
                memcpy(x, tmp.data(), nx*sizeof(double));
            }            
        }
        else {
            defaultInitialGuessState(this, nx, tau, x);
        }
    }
}


/**
 * 
 * This object will take responsibility for the memory and free it if the function is not longer needed.
 */
bool OptimalControlProblem::setInitialEstimate(functionLib::SingleVariableFunction* customInitialGuess, initialGuess::InitialGuessType type)
{
    functionLib::SingleVariableFunction** ptr;
    bool toStateWLagrange = false;
    if(customInitialGuess && customInitialGuess->NX() != 1) return false;
    switch(type){
        case initialGuess::InitialGuessType::Lagrange:  
            ptr = &this->customInitialGuessLagrange;
            if(customInitialGuess && customInitialGuess->NY() < 1) return false;
            break;
        case initialGuess::InitialGuessType::State:
            ptr = &this->customInitialGuessState;
            if(customInitialGuess && customInitialGuess->NY() < getNumberOfStateVariables()) return false;
            break;
        case initialGuess::InitialGuessType::Control:
            ptr = &this->customInitialGuessControl;
            if(customInitialGuess && customInitialGuess->NY() < getNumberOfControlVariables()) return false;
            break;
        case initialGuess::InitialGuessType::StateWithLagrange:
            ptr = &this->customInitialGuessState;
            toStateWLagrange = true;
            type = initialGuess::InitialGuessType::State;
            if(customInitialGuess && customInitialGuess->NY() < getNumberOfStateVariables()+1) return false;
            break;
        default:
            return false;
    }
    
    if( (!this->initialGuessIsStateLagrangeOneFunction && toStateWLagrange) ||
        (this->initialGuessIsStateLagrangeOneFunction && type == initialGuess::InitialGuessType::Lagrange) )
    { // special case if mixed case is set or was previously: delete state AND Lagrange
        if(this->customInitialGuessLagrange != 0) {
            delete (this->customInitialGuessLagrange);
            this->customInitialGuessLagrange = 0;
        }
        if(this->customInitialGuessState != 0) {
            delete (this->customInitialGuessState);
            this->customInitialGuessState = 0;
        }
    }
    if(*ptr != 0){
        delete (*ptr);
    }
    
    *ptr = customInitialGuess;
    
    if(toStateWLagrange && customInitialGuess == 0) {
        this->customInitialGuessLagrange = 0;
    }
    if(type != initialGuess::InitialGuessType::Control) this->initialGuessIsStateLagrangeOneFunction = toStateWLagrange;
    return true;
}


bool OptimalControlProblem::hasCustomInitialEstimate(const initialGuess::InitialGuessType type) const
{
    switch(type) {
        case initialGuess::InitialGuessType::State:
            return this->customInitialGuessState !=0;
        case initialGuess::InitialGuessType::Control:
            return this->customInitialGuessControl !=0;
        case initialGuess::InitialGuessType::Lagrange:
            return this->customInitialGuessLagrange || (this->customInitialGuessState !=0 && this->initialGuessIsStateLagrangeOneFunction);
        case initialGuess::InitialGuessType::StateWithLagrange:
            return this->customInitialGuessState && this->initialGuessIsStateLagrangeOneFunction;
        default:
            return false;
    }
}


bool OptimalControlProblem::setInitialTime(double initialTime){
    if(initialTime < this->tf) {
        this->t0 = initialTime;
        return true;
    }
    else {
        return false;
    }
}


const double OptimalControlProblem::getInitialTime() const
{
    return this->t0;
}


bool OptimalControlProblem::setFinalTime(double finalTime){
    if(this->t0 < finalTime) {
        this->tf = finalTime;
        return true;
    }
    else {
        return false;
    }
}


bool OptimalControlProblem::setFinalTime(double finalTime, double lowerB, double upperB){
    bool succ = true;
    succ &= this->setFinalTime(finalTime);
    if(lowerB <= this->tf) 
        this->tfLower = lowerB ;
    else {
        this->tfLower = this->tf;
        succ = false;
    }
    if(upperB >= this->tf) 
        this->tfUpper = upperB ;
    else {
        this->tfUpper = this->tf;
        succ = false;
    }
    return succ;
}


const double OptimalControlProblem::getFinalTime() const
{
    return this->tf;
}


void OptimalControlProblem::getFinalTime(double& finalTime, double& lowerB, double& upperB) const 
{
    finalTime = this->tf;
    lowerB = this->tfLower;
    upperB = this->tfUpper;
}


bool OptimalControlProblem::getFinalTimeIsFixed() const
{
    return (this->tfUpper-this->tfLower) < 1e-5;
}


bool OptimalControlProblem::setValuesAtInitialTime(const TVectorX& state, const TVectorX& control){
    bool retval = true;
    if(this->initialState.rows() == state.rows()){
        this->initialState = state;
        //stateIsActiveAtStart = VectorXb::Constant(initialState.size(), true);
    }
    else {
        std::cerr << "Illegal size of the given state vector.\n";
        retval = false;
    }
    if(this->initialControl.rows() == control.rows()) {
        this->initialControl = control;
        //controlIsActiveAtStart = VectorXb::Constant(initialControl.size(), true);
    }
    else {
        std::cerr << "Illegal size of the given control vector.\n";
        retval = false;
    }
    return retval;
}


bool OptimalControlProblem::setValuesAtFinalTime(const TVectorX& state, const TVectorX& control){
    bool retval = true;
    if(this->finalState.rows() == state.rows()){
        this->finalState = state;
    }
    else {
        std::cerr << "Illegal size of the given state vector.";
        retval = false;
    }
    if(this->finalControl.rows() == control.rows()){
        this->finalControl = control;
    }
    else {
        std::cerr << "Illegal size of the given control vector.";
        retval = false;
    }
    return retval;
}


void OptimalControlProblem::getStateAtInitialTime(TVectorX& istate)  const{
    istate = this->initialState;
}


void OptimalControlProblem::getControlAtInitialTime(TVectorX& icontrol)  const{
    icontrol = this->initialControl;
}


void ofc::models::OptimalControlProblem::getStateAtFinalTime(ofc::math::TVectorX& fstate) const
{
    fstate = this->finalState;
}


void OptimalControlProblem::getControlAtFinalTime(TVectorX& fcontrol)  const{
    fcontrol = this->finalControl;
}


bool OptimalControlProblem::setControlIsFixedAtFinalTime(const VectorXb& isfixed)
{
    if(isfixed.size() == this->controlIsFixedAtFinalTime.size()){
        this->controlIsFixedAtFinalTime = isfixed;
        return true;
    }
    else {
        std::cerr << "Invalid size of the given vector.\n";
        return false;
    }
}


bool OptimalControlProblem::setControlIsFixedAtInitialTime(const VectorXb& isfixed)
{
    if(isfixed.size() == this->controlIsFixedAtInitialTime.size()){
        this->controlIsFixedAtInitialTime = isfixed;
        return true;
    }
    else {
        std::cerr << "Invalid size of the given vector.\n";
        return false;
    }
}


void OptimalControlProblem::getControlIsFixedAtFinalTime(VectorXb& isfixed) const
{
    isfixed = this->controlIsFixedAtFinalTime;
}


void OptimalControlProblem::getControlIsFixedAtInitialTime(VectorXb& isfixed) const
{
    isfixed = this->controlIsFixedAtInitialTime;
}

void OptimalControlProblem::getStateIsFixedAtFinalTime(VectorXb& isfixed) const
{
    isfixed = this->stateIsFixedAtFinalTime;
}


void OptimalControlProblem::getStateIsFixedAtInitialTime(VectorXb& isfixed) const
{
    isfixed = this->stateIsFixedAtInitialTime;
}


bool OptimalControlProblem::setStateIsFixedAtFinalTime(const VectorXb& isfixed)
{
    if(isfixed.size() == this->stateIsFixedAtFinalTime.size()){
        this->stateIsFixedAtFinalTime = isfixed;
        return true;
    }
    else {
        std::cerr << "Invalid size of the given vector.\n";
        return false;
    }
}


bool OptimalControlProblem::setStateIsFixedAtInitialTime(const VectorXb& isfixed)
{
    if(isfixed.size() == this->stateIsFixedAtInitialTime.size()) {
        this->stateIsFixedAtInitialTime = isfixed;
        return true;
    }
    else {
        std::cerr << "Invalid size of the given vector.\n";
        return false;
    }
}


void OptimalControlProblem::getControlVariableLowerBound(TVectorX& controlLB)  const{
    controlLB = this->controlVariableLowerBound;
}


void OptimalControlProblem::getControlVariableUpperBound(TVectorX& controlUB)  const{
    controlUB = this->controlVariableUpperBound;
}


void OptimalControlProblem::getStateVariableLowerBound(TVectorX& stateLB)  const{
    stateLB = this->stateVariableLowerBound;
}


void OptimalControlProblem::getStateVariableUpperBound(TVectorX& stateUB)  const{
    stateUB = this->stateVariableUpperBound;
}


bool ofc::models::OptimalControlProblem::getStateVariableBounds(double& lower, double& upper, const size_t index) const {
    if(index < this->getNumberOfStateVariables()){
        lower = this->stateVariableLowerBound[index];
        upper = this->stateVariableUpperBound[index];
        return true;
    }
    return false;
}


bool ofc::models::OptimalControlProblem::getControlVariableBounds(double& lower, double& upper, const size_t index) const {
    if(index < this->getNumberOfControlVariables()){
        lower = this->controlVariableLowerBound[index];
        upper = this->controlVariableUpperBound[index];
        return true;
    }
    return false;    
}


void OptimalControlProblem::setControlVariableLowerBound(const TVectorX& controlLB){
    if(controlLB.rows() == this->controlVariableLowerBound.rows())
        this->controlVariableLowerBound = controlLB;
    else
        std::cerr << "Invalid size of the given vector.\n";
}


void OptimalControlProblem::setControlVariableUpperBound(const TVectorX& controlUB){
    if(controlUB.rows() == this->controlVariableUpperBound.rows())
        this->controlVariableUpperBound = controlUB;
    else
        std::cerr << "Invalid size of the given vector.\n";
}


void OptimalControlProblem::setStateVariableLowerBound(const TVectorX& stateLB){
    if(stateLB.rows() == this->stateVariableLowerBound.rows())
        this->stateVariableLowerBound = stateLB;
    else
        std::cerr << "Invalid size of the given vector.\n";
}


void OptimalControlProblem::setStateVariableUpperBound(const TVectorX& stateUB){
    if(stateUB.rows() == this->stateVariableUpperBound.rows())
        this->stateVariableUpperBound = stateUB;
    else
        std::cerr << "Invalid size of the given vector.\n";
}


void OptimalControlProblem::getControlIsUnconstrainedAngle(VectorXb& isunconstrained) const{
    isunconstrained = this->controlIsUnconstrainedAngle;
}


void OptimalControlProblem::getStateIsUnconstrainedAngle(VectorXb& isunconstrained) const{
    isunconstrained = this->stateIsUnconstrainedAngle;
}


bool OptimalControlProblem::setControlIsUnconstrainedAngle(const VectorXb& in){
    if(in.rows() != this->controlIsUnconstrainedAngle.rows())
        return false;
    this->controlIsUnconstrainedAngle = in;
    for(size_t i = 0; i < this->controlVariableLowerBound.size(); i++){
        if(this->controlIsUnconstrainedAngle[i]){
            this->controlVariableLowerBound[i] = -std::numeric_limits<double>::infinity(); // set respective limits to infinity
            this->controlVariableUpperBound[i] = std::numeric_limits<double>::infinity();
        }
    }
    return true;
}


bool OptimalControlProblem::setStateIsUnconstrainedAngle(const VectorXb& in){
    if(in.rows() != this->stateIsUnconstrainedAngle.rows())
        return false;
    this->stateIsUnconstrainedAngle = in;
    for(size_t i = 0; i < this->stateVariableLowerBound.size(); i++){
        if(this->stateIsUnconstrainedAngle[i]){
            this->stateVariableLowerBound[i] = -std::numeric_limits<double>::infinity(); // set respective limits to infinity
            this->stateVariableUpperBound[i] = std::numeric_limits<double>::infinity();
        }
    }
    return true;
}


void OptimalControlProblem::setProblemName(const std::string name){
    this->problemName = name;
}


const void OptimalControlProblem::getProblemName(std::string& name) const{
    name = this->problemName;
}


std::string OptimalControlProblem::getStateVariableName(int i)  const{
    return std::string("state ")+std::to_string(i);
}

std::string OptimalControlProblem::getControlVariableName(int i) const{
    return std::string("control ")+std::to_string(i);
}


std::string OptimalControlProblem::getConstantDescriptions(bool printOnConsole) const {
    std::stringstream ss;
    for(int i = 0; i < getNumberOfConstants(); i++){
        ss << "Constant " << i << ": " << constantsDescription[i] << ": " << constants[i] << std::endl;
    }
    if(printOnConsole)
        std::cout << ss.str();
    return ss.str();
}


std::string OptimalControlProblem::getConstantDescription(size_t index) const {
    if(index < getNumberOfConstants()) {
        return constantsDescription[index];
    }
    return "";
}


bool OptimalControlProblem::setConstant(int index, double value){
    if(index < 0 || index >= getNumberOfConstants()){
        std::cerr << "OptimalFeedbackControlProblem:setConstant: Invalid index. Index must be nonnegative and strictly smaller than " << getNumberOfConstants() << "." << std::endl;
        return false;
    }
    this->constants[index] = value;
    return true;
}


bool ofc::models::OptimalControlProblem::setConstant(const std::string descr, double value)
{
    std::vector<std::string>::iterator it = std::find(constantsDescription.begin(), constantsDescription.end(), descr);
    if(it == constantsDescription.end()) {
        std::cerr << "OptimalFeedbackControlProblem:getConstant: Invalid constant name." << std::endl;
        return false;
    }
    constants[std::distance(constantsDescription.begin(), it)] = value;
    return true;
}

bool ofc::models::OptimalControlProblem::isConstant(const std::string& descr) const {
    std::vector<std::string>::const_iterator it = std::find(constantsDescription.begin(), constantsDescription.end(), descr);
    return (it != constantsDescription.end());
}


double OptimalControlProblem::getConstant(int index) const
{
    if(index < 0 || index >= getNumberOfConstants()){
        std::cerr << "OptimalFeedbackControlProblem:getConstant: Invalid index. Index must be nonnegative and strictly smaller than " << getNumberOfConstants() << "." << std::endl;
        return std::numeric_limits<double>::infinity();
    }
    
    if(externalConstantsArray && externalConstantsArray->size() >= getNumberOfConstants())
        return (*externalConstantsArray)[index];
    else
        return this->constants[index];
}


double ofc::models::OptimalControlProblem::getConstant(const std::string descr) const
{
    std::vector<std::string>::const_iterator it = std::find(constantsDescription.begin(), constantsDescription.end(), descr);
    if(it == constantsDescription.end()) {
        std::cerr << "OptimalFeedbackControlProblem:getConstant: Invalid constant name." << std::endl;
        return std::numeric_limits<double>::infinity();
    }
    return constants[std::distance(constantsDescription.begin(), it)];
}


const double* OptimalControlProblem::getConstants() const {
    if(externalConstantsArray && externalConstantsArray->size() >= getNumberOfConstants())
        return &(*externalConstantsArray)[0];
    else
        return &this->constants[0];
}


void OptimalControlProblem::addConstant(double defaultValue, std::string description)
{
    this->constants.push_back(defaultValue);
    this->constantsDescription.push_back(description);
}


bool OptimalControlProblem::removeConstant(int index)
{
    if(index < 0 || index >= getNumberOfConstants()){
        return false;
    }
    this->constants.erase(this->constants.begin() + (index-1));
    this->constantsDescription.erase(this->constantsDescription.begin() + (index-1));
    return true;
}


const double * ofc::models::OptimalControlProblem::getParameters() const {
    return &this->parameters[0];
}


double ofc::models::OptimalControlProblem::getParameter(int index) const {
    if(index < 0 || index >= getNumberOfParameters()){
        std::cerr << "OptimalFeedbackControlProblem:getParameter: Invalid index. Index must be nonnegative and strictly smaller than " << getNumberOfParameters() << "." << std::endl;
        return std::numeric_limits<double>::infinity();
    }
    return this->parameters[index];
}

double ofc::models::OptimalControlProblem::getParameter(const std::string descr) const
{
    std::vector<std::string>::const_iterator it = std::find(parameterDescription.begin(), parameterDescription.end(), descr);
    if(it == parameterDescription.end()) {
        std::cerr << "OptimalFeedbackControlProblem:getParameter: Invalid parameter name." << std::endl;
        return std::numeric_limits<double>::infinity();
    }
    return parameters[std::distance(parameterDescription.begin(), it)];
}


void ofc::models::OptimalControlProblem::addParameter(double defaultValue, std::string description) {
    this->parameters.push_back(defaultValue);
    this->parameterDescription.push_back(description);
}


bool ofc::models::OptimalControlProblem::removeParameter(int index) {
    if(index < 0 || index >= getNumberOfParameters()){
        return false;
    }
    this->parameters.erase(this->parameters.begin() + (index-1));
    this->parameterDescription.erase(this->parameterDescription.begin() + (index-1));
    return true;
}


bool ofc::models::OptimalControlProblem::setParameter(int index, double value) {
    if(index < 0 || index >= getNumberOfParameters()){
        std::cerr << "OptimalFeedbackControlProblem:setParameter: Invalid index. Index must be nonnegative and strictly smaller than " << getNumberOfParameters() << "." << std::endl;
        return false;
    }
    this->parameters[index] = value;
    return true;
}


bool ofc::models::OptimalControlProblem::setParameter(const std::string descr, double value)
{
    std::vector<std::string>::iterator it = std::find(parameterDescription.begin(), parameterDescription.end(), descr);
    if(it == parameterDescription.end()) {
        std::cerr << "OptimalFeedbackControlProblem:getParameter: Invalid parameter name." << std::endl;
        return false;
    }
    parameters[std::distance(parameterDescription.begin(), it)] = value;
    return true;
}

bool ofc::models::OptimalControlProblem::isParameter(const std::string& descr) const
{
    std::vector<std::string>::const_iterator it = std::find(parameterDescription.begin(), parameterDescription.end(), descr);
    return (it != parameterDescription.end());
}



std::string ofc::models::OptimalControlProblem::getParameterDescription(size_t index) const {
    if(index < getNumberOfParameters()) {
        return parameterDescription[index];
    }
    return "";
}


std::string ofc::models::OptimalControlProblem::getParameterDescriptions(bool printOnConsole) const {
    std::stringstream ss;
    for(int i = 0; i < getNumberOfParameters(); i++){
        ss << "Constant " << i << ": " << parameterDescription[i] << ": " << parameters[i] << std::endl;
    }
    if(printOnConsole)
        std::cout << ss.str();
    return ss.str();
}


bool ofc::models::OptimalControlProblem::setExternalConstantsArray(std::vector<double>* eca)
{
    if(!eca) {
        externalConstantsArray = 0;
        return true;
    }
    else if(eca->size() == getNumberOfConstants()){
        externalConstantsArray = eca;
        return true;
    }
    return false;
}


void ofc::models::OptimalControlProblem::addDynamicMode(std::string description)
{
    this->dynamicModeDescription.push_back(description);
}

bool ofc::models::OptimalControlProblem::setDynamicModeDescription(const ofc::models::uint id, const std::string description)
{
    if(id < this->dynamicModeDescription.size()) {
        this->dynamicModeDescription[id] = description;
        return true;
    }
    else return false;
}


