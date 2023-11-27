#pragma once

#include <istream>
#include <ostream>
#include <vector>
#include <Eigen/Eigen>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/export.hpp>

#include "../math/definitions.h"
#include "../serialization/eigen_dense_serialization.h"

#include "../functionReimplement/SingleVariableFunction.h"

namespace ofc::models {
    
    
    namespace initialGuess {
        enum InitialGuessType{
            Lagrange=0,
            State=1,
            Control=2,
            StateWithLagrange=3,
        };
    }
    
    typedef unsigned int uint;
    
    /**
     * 
     * 
     */
    class OptimalControlProblem {
    private:
        friend class boost::serialization::access;
        std::vector<double> constants;
        std::vector<std::string> constantsDescription;
        std::vector<double> parameters;
        std::vector<std::string> parameterDescription;
        uint dynamicMode;
        std::vector<std::string> dynamicModeDescription;
        std::vector<double>* externalConstantsArray;
    protected:
        
        std::string problemName;
        
        math::TVectorX finalControl;
        math::TVectorX finalState;
        math::TVectorX initialControl;
        math::TVectorX initialState;
        
        double t0;
        double tf;
        double tfLower;
        double tfUpper;
        
        math::TVectorX stateVariableLowerBound;
        math::TVectorX stateVariableUpperBound;
        math::VectorXb stateIsFixedAtInitialTime;
        math::VectorXb stateIsFixedAtFinalTime;
        
        math::TVectorX controlVariableLowerBound;
        math::TVectorX controlVariableUpperBound;
        math::VectorXb controlIsFixedAtInitialTime;
        math::VectorXb controlIsFixedAtFinalTime;
        
        math::VectorXb stateIsUnconstrainedAngle;
        math::VectorXb controlIsUnconstrainedAngle;
        
        functionLib::SingleVariableFunction* customInitialGuessLagrange;
        functionLib::SingleVariableFunction* customInitialGuessState;
        functionLib::SingleVariableFunction* customInitialGuessControl;
        bool initialGuessIsStateLagrangeOneFunction;
        
        void addParameter(double defaultValue, std::string description);
        bool removeParameter(int index);
        
        void addConstant(double defaultValue, std::string description);
        bool removeConstant(int index);
        
        void addDynamicMode(std::string description);
        bool setDynamicModeDescription(const uint id, const std::string description);
        
        void build(const uint nx, const uint nu);
        void build();
        
    public:
        // construct, destruct, save and load object:
        OptimalControlProblem();
        OptimalControlProblem(const uint nx, const uint nu);
        OptimalControlProblem (const OptimalControlProblem& p);
        virtual OptimalControlProblem* copy() const;
        virtual ~OptimalControlProblem();
        
        static void serializeOCP(const OptimalControlProblem* ocp, std::ostream& s);  
        static bool unserializeOCP( OptimalControlProblem** ocp, std::istream& s);
        std::string serialize() const;
        static OptimalControlProblem* unserialize(const std::string& s);
        
        typedef OptimalControlProblem* create_default_t();
        typedef OptimalControlProblem* create_param_t(const uint nx, const uint nu);
        typedef OptimalControlProblem* create_copy_t(const OptimalControlProblem& p);
        typedef OptimalControlProblem* create_unserialize_t(std::istream& s);
        typedef void destroy_t(OptimalControlProblem*);
        
        virtual void initialize() {};
        
        // problem-specific functions:
        virtual bool objectiveMayerTerm(double& fobj, double tf, uint nx, uint nu, uint np, const double* x0, const double* u0, const double* p, const double* xf, const double* uf) const {fobj = 0; return true;};
        virtual bool objectiveLagrangeTermIntegrand(double& fobj, uint nx, uint nu, uint np, const double* x, const double* u, const double* p) const {fobj = 0; return true;};
        virtual bool evaluate(double* f, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {for(size_t i = 0; i < nx; ++i){f[i] = 0;} return true;};
        
        virtual bool NonlinearEqualityConstraints(double* nec, const double *state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {return false;};
        virtual bool NonlinearInequalityConstraints(double* nec, const double *state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {return false;};
        virtual bool NonlinearBoundaryConditions(double* xf, double* uf, const double* x0, const double* u0, const double* p, double t0, double tf, uint nx, uint nu, uint np) const {return false;};
        virtual bool implicitNonlinearBoundaryConditions(double* rb, const double* x0, const double* u0, const double* p, double* xf, double* uf, double t0, double tf, uint nx, uint nu, uint np, uint nrb) const {return false;};
        
        // derivatives:
        virtual bool dynamicsJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {return false;};
        virtual bool objectiveLagrangeJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {return false;};
        virtual bool objectiveMayerJacobians(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, const double* state_t0, const double* control_t0, const double* parameter, const double tf, const double* state_tf, const double* control_tf, uint nx, uint nu, uint np) const {return false;};
        
        
        virtual bool NECJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {return false;};
        virtual bool NICJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const {return false;};
        virtual bool NBCJacobians(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, uint nx, uint nu, uint np, const double* x0, const double* u0, const double* p, double t0, double tf, double* xf, double* uf) const {return false;};
        
        // get dimensions:
        int getNumberOfStateVariables() const;
        int getNumberOfControlVariables() const;
        int getNumberOfParameters() const;
        int getNumberOfConstants() const;
        virtual int getNumberOfInequalityConstraints() const {return 0;};
        virtual int getNumberOfEqualityConstraints() const {return 0;};
        virtual int getNumberOfImplicitNonlinearBoundaryConditions() const {return 0;};
        
        uint getDynamicMode() const {return this->dynamicMode;};
        bool setMode(const uint mode) {if(mode < this->dynamicModeDescription.size()) {this->dynamicMode = mode; 
            build(); return true;} else return false;};
        std::string modeDescription() const {return this->dynamicModeDescription[this->dynamicMode];};
        
        // initial estimate:
        void initialEstimate(int nx, int lu, double tau, double* x, double* u, double& l) const;
        bool hasCustomInitialEstimate(const initialGuess::InitialGuessType type) const;
        bool setInitialEstimate(functionLib::SingleVariableFunction* customInitialGuess, initialGuess::InitialGuessType type);
        
        // initial- and final time:
        bool setInitialTime(double initialTime);
        const double getInitialTime() const;
        bool setFinalTime(double finalTime);
        bool setFinalTime(double finalTime, double lowerB, double upperB);
        const double getFinalTime() const;
        void getFinalTime(double& finalTime, double& lowerB, double& upperB) const;
        bool getFinalTimeIsFixed() const;
        
        // initial and final values (state and control):
        bool setValuesAtInitialTime(const math::TVectorX& state, const math::TVectorX& control);
        bool setValuesAtFinalTime(const math::TVectorX& state, const math::TVectorX& control);
        void getStateAtInitialTime(math::TVectorX& istate) const;
        void getStateAtFinalTime(math::TVectorX& fstate)  const;
        void getControlAtInitialTime(math::TVectorX& icontrol)  const;
        void getControlAtFinalTime(math::TVectorX& fcontrol)  const;
        
        bool setStateIsFixedAtInitialTime(const math::VectorXb& isfixed);
        bool setStateIsFixedAtFinalTime(const math::VectorXb& isfixed);
        bool setControlIsFixedAtInitialTime(const math::VectorXb& isfixed);
        bool setControlIsFixedAtFinalTime(const math::VectorXb& isfixed);
        void getStateIsFixedAtInitialTime(math::VectorXb& isfixed) const;
        void getStateIsFixedAtFinalTime(math::VectorXb& isfixed) const;
        void getControlIsFixedAtInitialTime(math::VectorXb& isfixed) const;
        void getControlIsFixedAtFinalTime(math::VectorXb& isfixed) const;
        
        // bounds on state and control variable:
        void getStateVariableLowerBound(math::TVectorX &stateLB) const;
        void getStateVariableUpperBound(math::TVectorX &stateUB)  const;
        void getControlVariableLowerBound(math::TVectorX &controlLB) const;
        void getControlVariableUpperBound(math::TVectorX &controlUB) const;
        bool getStateVariableBounds(double& lower, double& upper, const size_t index) const;
        bool getControlVariableBounds(double& lower, double& upper, const size_t index) const;
        
        void setStateVariableLowerBound(const math::TVectorX& stateLB);
        void setStateVariableUpperBound(const math::TVectorX& stateUB);
        void setControlVariableLowerBound(const math::TVectorX& controlLB);
        void setControlVariableUpperBound(const math::TVectorX& controlUB);
        
        // unconstrained angle:
        bool setStateIsUnconstrainedAngle(const math::VectorXb& in);
        bool setControlIsUnconstrainedAngle(const math::VectorXb& in);
        void getStateIsUnconstrainedAngle(math::VectorXb& isunconstrained) const;
        void getControlIsUnconstrainedAngle(math::VectorXb& isunconstrained) const;
        
        // names:
        void setProblemName(const std::string name);
        const void getProblemName(std::string &name) const;
        virtual std::string getStateVariableName(int i) const;
        virtual std::string getControlVariableName(int i) const;
        
        // constants and parameter
        virtual std::string getConstantDescriptions(bool printOnConsole = false) const;
        virtual std::string getConstantDescription(size_t index) const;
        virtual std::string getParameterDescriptions(bool printOnConsole = false) const;
        virtual std::string getParameterDescription(size_t index) const;
        bool setParameter(int index, double value);
        bool setParameter(const std::string descr, double value);
        double getParameter(int index) const;
        double getParameter(const std::string descr) const;
        bool isParameter(const std::string& descr) const;
        bool setConstant(int index, double value);
        bool setConstant(const std::string descr, double value);
        double getConstant(int index) const;
        double getConstant(const std::string descr) const;
        bool isConstant(const std::string& descr) const;
        
        const double* getParameters() const;
        const double* getConstants() const;
        
        bool setExternalConstantsArray(std::vector<double>* eca);
        
        virtual std::string getThisLibraryPathname() const;
    private:
        
        template<class Archive> 
        void save(Archive& ar, const unsigned int) const
        {            
            ar & constants;
            ar & constantsDescription;
            ar & parameters;
            ar & parameterDescription;
            ar & dynamicMode;
            ar & dynamicModeDescription;
            
            ar & problemName;
            
            ar & finalControl;
            ar & finalState;
            ar & initialControl;
            ar & initialState;
            
            ar & t0;
            ar & tf;
            ar & tfLower;
            ar & tfUpper;
            
            ar & stateVariableLowerBound;
            ar & stateVariableUpperBound;
            ar & stateIsFixedAtInitialTime;
            ar & stateIsFixedAtFinalTime;
            
            ar & controlVariableLowerBound;
            ar & controlVariableUpperBound;
            ar & controlIsFixedAtInitialTime;
            ar & controlIsFixedAtFinalTime;
            
            ar & stateIsUnconstrainedAngle;
            ar & controlIsUnconstrainedAngle;
            
            ar & customInitialGuessLagrange;
            ar & customInitialGuessControl;
            ar & customInitialGuessState;
            ar & initialGuessIsStateLagrangeOneFunction;     
        }
        
        template<class Archive>
        void load(Archive& ar, const unsigned int)
        {            
            ar & constants;
            ar & constantsDescription;
            ar & parameters;
            ar & parameterDescription;
            ar & dynamicMode;
            ar & dynamicModeDescription;
            
            ar & problemName;
            
            ar & finalControl;
            ar & finalState;
            ar & initialControl;
            ar & initialState;
            
            ar & t0;
            ar & tf;
            ar & tfLower;
            ar & tfUpper;
            
            ar & stateVariableLowerBound;
            ar & stateVariableUpperBound;
            ar & stateIsFixedAtInitialTime;
            ar & stateIsFixedAtFinalTime;
            
            ar & controlVariableLowerBound;
            ar & controlVariableUpperBound;
            ar & controlIsFixedAtInitialTime;
            ar & controlIsFixedAtFinalTime;
            
            ar & stateIsUnconstrainedAngle;
            ar & controlIsUnconstrainedAngle;
            
            ar & customInitialGuessLagrange;
            ar & customInitialGuessControl;
            ar & customInitialGuessState;
            ar & initialGuessIsStateLagrangeOneFunction;            
        }
        
        BOOST_SERIALIZATION_SPLIT_MEMBER()
        
    };
    
}

BOOST_CLASS_EXPORT_KEY(ofc::models::OptimalControlProblem)
