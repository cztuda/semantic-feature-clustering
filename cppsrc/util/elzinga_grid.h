#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>


#define svr_sym int
#define svr_num int
#define svr_val double

typedef double (*lwptr)(const svr_num& k, const double& mue_k);
typedef double (*gwptr)(const svr_num&, const svr_num&, const svr_num&, const svr_num&);

struct SVRspellOptions{
public:    
    SVRspellOptions(const Eigen::MatrixXd& softMatchingMatrix=Eigen::MatrixXd(0,0),
                      const Eigen::VectorXd& weights=Eigen::VectorXd(0),
                      const lwptr lengthWeighting=nullptr,
                      const gwptr gapWeighting=nullptr,
                      const gwptr gapWeighting_xx=nullptr,
                      const gwptr gapWeighting_xy=nullptr,
                      const std::vector<svr_val>& tx=std::vector<svr_val>(), 
                      const std::vector<svr_val>& ty=std::vector<svr_val>(),
                      const std::vector<svr_val>& vx=std::vector<svr_val>(), 
                      const std::vector<svr_val>& vy=std::vector<svr_val>()
                   );
//     SVRspellOptions()=delete;
//     SVRspellOptions(const SVRspellOptions& options)=delete;
//     SVRspellOptions & operator= (const SVRspellOptions& options)=delete;
    
    bool prepareOptions(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y);
    
    const Eigen::MatrixXd& softMatching() const;
    const Eigen::VectorXd& charWeights() const;
    lwptr lengthWeighting() const;
    gwptr gapWeighting() const;
    const std::vector<svr_val>& tx() const; 
    const std::vector<svr_val>& ty() const;
    const std::vector<svr_val>& vx() const; 
    const std::vector<svr_val>& vy() const;
    bool isPrepared() const;
    bool useRunLength() const;
    bool useValues() const;
    bool useModeSpecificGapWeight() const;
    
    enum class MODE {XY, XX, YY};
    MODE mode() const;
    void mode(const MODE mode) const;
private:
    bool _is_prepared;
    bool _use_run_length;
    bool _use_mode_specific_gap_weight;
    bool _use_values;
    mutable MODE _mode;
    
    Eigen::MatrixXd _softMatching;
    Eigen::VectorXd _charWeights;
    double (*_lengthWeighting)(const svr_num& k, const double& mue_k);
    double (*_gapWeighting)(const svr_num&, const svr_num&, const svr_num&, const svr_num&);
    gwptr _gapWeighting_xx;
    gwptr _gapWeighting_yy;
    std::vector<svr_val> _tx; 
    std::vector<svr_val> _ty;
    std::vector<svr_val> _vx;
    std::vector<svr_val> _vy;
};



/*
 * Evaluate kernels for the function f(x,u):=|x|_u (if u \subseq x)
 * 
 * - length weighting function must be convex!
 * - soft matching weight matrix must be symmetric and positive definite
 * - if weights and a soft matching matrix are given, the weights overwrite the diagonal of the soft matching matrix
 * - set dimensions of soft matching matrix or weights vector to zero to disable
 * - length of tx and ty must match the length of x and y respectively
 */
double grid_algorithm(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const SVRspellOptions& options);



/*
 * Kernel function.
 * 
 * - soft-matching weight matrix (nAlphabet^2): symmetric, positive semi-definite
 * - character weights vector (nAlphabet): if weights and a soft matching matrix are given, the weights overwrite the diagonal of the soft matching matrix
 * - lengthWeighting: convex function
 * - gapWeighting: arbitrary function
 * - tx, ty: vectors (|x| and |y| respectively)
 * 
 * Set functions to nullptr and vectors/matrices to length=0 to disable
 */
double trail_algorithm(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const SVRspellOptions& options);



double SVRspell(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, const SVRspellOptions& options);


    
double distance(const std::vector<svr_sym>& x, const std::vector<svr_sym>& y, SVRspellOptions& options, double& xy, double& xx, double& yy);
