#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

double polyeval(Eigen::VectorXd coeffs, double x);

class MPC {
public:
	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

	std::vector<double> x_predicted_;
	std::vector<double> y_predicted_;
	std::vector<double> last_actuators_;

	static size_t N;
	static double dt;
	static const double delay_s;
	static const size_t NUM_ACTUATIONS;
	static size_t delay_steps_;
};

#endif /* MPC_H */
