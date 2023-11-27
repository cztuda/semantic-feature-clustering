
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <chrono>

#include <boost/numeric/odeint.hpp>
#include <boost/iterator/counting_iterator.hpp>

using namespace std;
using namespace Eigen;

using namespace boost::numeric::odeint;

inline void printsep(){
    cout << endl << "----------------------------------------" << endl << endl;
}


typedef std::vector< double > state_type;

const double sigma = 10.0;
const double R = 28.0;
const double b = 8.0 / 3.0;

// the system function can be a classical functions
void lorenz( state_type &x , state_type &dxdt , double t )
{                                                         
    dxdt[0] = sigma * ( x[1] - x[0] );
    dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
    dxdt[2] = x[0]*x[1] - b * x[2];
}

// the system function can also be a functor
class lorenz_class
{
 public:
    void operator()( state_type &x , state_type &dxdt , double t )
    {
        dxdt[0] = sigma * ( x[1] - x[0] );
        dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
        dxdt[2] = x[0]*x[1] - b * x[2];
    }
};

struct streaming_observer
{
     std::ostream &m_out;
     streaming_observer( std::ostream &out ) : m_out( out ) {}

     void operator()( const state_type &x , double t ) const
     {
          m_out << t;
          for( size_t i=0 ; i < x.size() ; ++i )
              m_out << "\t" << x[i];
          m_out << "\n";
     }
};


/*
void harmonic_oscillator(const Eigen::VectorXd &x, Eigen::VectorXd &dxdt, const double t){
    const double gam = 0.15;
            
    dxdt[0] = x[1];
    dxdt[1] = -x[0] - gam*x[1];
};

void printer(const Eigen::VectorXd &x, const double t)
{
    std::cout << "time: " << t << " state: " << x << std::endl;
};*/

void helper(const Eigen::VectorXd& x0, double t0, double t1, Eigen::VectorXd& xf);

    typedef controlled_runge_kutta< runge_kutta_dopri5< state_type > > dopri_stepper_type;
    typedef dense_output_runge_kutta< dopri_stepper_type > dense_stepper_type;
    
int main() {
    const MatrixXd b = MatrixXd::Random(1,3);
    
    cout << b << endl;    
    printsep();
    
    auto start = chrono::steady_clock::now();
    cout << 113483257012.34329*(b.transpose()*b).array() + 6 << endl;
    auto end = chrono::steady_clock::now();
    
    cout<<chrono::duration_cast<chrono::microseconds>(end-start).count()<< " microseconds" << endl;
	cout<<chrono::duration_cast<chrono::milliseconds>(end-start).count()<<" milliseconds"<<endl;
    
//     cout << A << endl;
    printsep();
    
    state_type x( 3 );
    x[0] = x[1] = x[2] = 10.0;
    const double dt = 0.01;
    
    state_type x2 = x;

    
    double abs_error = 1.0e-10 , rel_error = 1.0e-10;
    integrate_const( dense_stepper_type(default_error_checker< double, range_algebra, default_operations >( abs_error , rel_error )) , lorenz , x , 0.0 , 10.0 , dt , streaming_observer( std::cout ) );
    
    std::cout << "\nintegrate_times:" << std::endl;
    
    
    Eigen::VectorXd x0;
    x0.resize(3);
    x0 << 10, 10, 10;
    Eigen::VectorXd xf;
    xf.resize(3);
    helper(x0, 0, 10, xf);
    return 0;
    
    std::vector<double> L;
    for(int i = 0; i <= 10; ++i){
        L.push_back(static_cast<double>(i));
    }
    
    
    integrate_times( dense_stepper_type(default_error_checker< double, range_algebra, default_operations >( abs_error , rel_error )) , lorenz , x2 , L.begin() , L.end() , dt , streaming_observer( std::cout ) );
    
    // or use the functor:
    // integrate_const( runge_kutta4< state_type >() , lorenz_class() , x , 0.0 , 10.0 , dt );
    
}

struct push_back_time
{
    Eigen::VectorXd* xf;
    bool hasBeenCalled;

    push_back_time( Eigen::VectorXd* xf ) : hasBeenCalled(false) { this->xf = xf; }

    void operator()( const state_type &x , double t )
    {
        if(hasBeenCalled){
            std::cout << "time = " << t << std::endl;
            xf->resize(x.size());
            for(int i = 0; i < x.size(); ++i){
                (*xf)[i] = x[i];
                std::cout << "xf[" << i << "] = " << x[i] << std::endl;
            }            
        }
        else {
            hasBeenCalled = true;
        }
    }
};

void helper(const Eigen::VectorXd& x0, double t0, double t1, Eigen::VectorXd& xf){
    std::vector<double> tf_tmp_list;
    tf_tmp_list.push_back(t0);
    tf_tmp_list.push_back(t1);
    
    auto odeintegr = dense_stepper_type(default_error_checker< double, range_algebra, default_operations >( 1e-10 , 1e-10 ));
    
    std::vector<double> times;
    auto xStart = std::vector<double>(x0.begin(), x0.end());
    
    boost::numeric::odeint::integrate_times( odeintegr, lorenz, xStart , tf_tmp_list.begin(), tf_tmp_list.end() , 1e-4 , push_back_time(&xf));
    
}
