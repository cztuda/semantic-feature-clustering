#pragma once

#include "../optimalcontrolproblem.h"
#include <cppad/cg.hpp>


namespace ofc::models {
    
    class Furutapendulum_smoothProblem : public OptimalControlProblem
    {
    public:
        Furutapendulum_smoothProblem();
        Furutapendulum_smoothProblem(const Furutapendulum_smoothProblem& p);
        ~Furutapendulum_smoothProblem();
        void initialize() override;
        
        virtual bool objectiveMayerTerm(double& fobj, double tf, uint nx, uint nu, uint np, const double* x0, const double* u0, const double* p, const double* xf, const double* uf) const override;
        virtual bool objectiveLagrangeTermIntegrand(double& fobj, uint nx, uint nu, uint np, const double* x, const double* u, const double* p) const override;
        virtual bool evaluate(double* f, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        
        virtual bool NonlinearEqualityConstraints(double* nec, const double *state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        virtual bool NonlinearInequalityConstraints(double* nec, const double *state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        virtual bool NonlinearBoundaryConditions(double* xf, double* uf, const double* x0, const double* u0, const double* p, double t0, double tf, uint nx, uint nu, uint np) const override;
        virtual bool implicitNonlinearBoundaryConditions(double* rb, const double* x0, const double* u0, const double* p, double* xf, double* uf, double t0, double tf, uint nx, uint nu, uint np, uint nrb) const override;
        
        // derivatives:
        virtual bool dynamicsJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        virtual bool objectiveLagrangeJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        virtual bool objectiveMayerJacobians(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, const double* state_t0, const double* control_t0, const double* parameter, const double tf, const double* state_tf, const double* control_tf, uint nx, uint nu, uint np) const override;
        virtual bool NECJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        virtual bool NICJacobians(double* J_x, double* J_u, double* J_c, double* J_p, const double* state, const double* control, const double* parameter, uint nx, uint nu, uint np) const override;
        virtual bool NBCJacobians(double* J_x0, double* J_u0, double* J_c, double* J_p, double* J_xf, double* J_uf, double* J_tf, uint nx, uint nu, uint np, const double* x0, const double* u0, const double* p, double t0, double tf, double* xf, double* uf) const override;
        
        
        OptimalControlProblem* copy() const override
        {
            OptimalControlProblem* o = new Furutapendulum_smoothProblem(*this);
            return o;
        }
        
        virtual int getNumberOfInequalityConstraints() const override;
        virtual int getNumberOfEqualityConstraints() const override;
        virtual int getNumberOfImplicitNonlinearBoundaryConditions() const override;
        
        virtual std::string getThisLibraryPathname() const override;
    protected:
        static long derivativeLibraryInstanceCounter;
        static std::unique_ptr<CppAD::cg::LinuxDynamicLib<double>> derivativeLibrary;
        static void manageDerivativeLibrary();
    private:
        friend class boost::serialization::access;
        template<class Archive> 
        void save(Archive& ar, const unsigned int) const
        {
            ar & boost::serialization::base_object<OptimalControlProblem>(*this);
        }
        
        template<class Archive>
        void load(Archive& ar, const unsigned int)
        {
            ar & boost::serialization::base_object<OptimalControlProblem>(*this);
        }
        
        BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
    
}

BOOST_CLASS_EXPORT_KEY(ofc::models::Furutapendulum_smoothProblem)
