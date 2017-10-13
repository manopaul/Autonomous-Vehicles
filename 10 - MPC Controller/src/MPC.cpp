#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

using CppAD::AD;

// Set the timestep length and duration
size_t N = 7; // Setting this number to be greater than 12 makes car stop aronud curve
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// TODO: Set Reference velocity
double MAX_VELOCITY = 80;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    const int REF_STATE_FACTOR = 600;
    const int ACTUATOR_FACTOR = 10;
    const int COMPENSATING_FACTOR = 100;
    const int STEERING_ANGLE_DELTA_FACTOR = 25;
    const int ACCELERATION_DELTA_FACTOR = 10;

    // Reference State cost
    for(int t=0; t < N; t++) {
      fg[0] += REF_STATE_FACTOR * CppAD::pow(vars[cte_start + t], 2); // CTE error
      fg[0] += REF_STATE_FACTOR * CppAD::pow(vars[epsi_start + t], 2); // Heading (Orientation Angle) error
      fg[0] += CppAD::pow(vars[v_start + t] - MAX_VELOCITY, 2); // Velocity error
    }

    // Minimize the use of actuators
    for(int t=0; t < N - 1; t++) {
      fg[0] += ACTUATOR_FACTOR * CppAD::pow(vars[delta_start + t], 2); // Steering Angle actuator
      fg[0] += ACTUATOR_FACTOR * CppAD::pow(vars[a_start + t], 2); //Acceleration/Braking actuator; + implies acceleration; -ve implies Braking
      //Adding penalty to handle steering and throttle (Speed)
      fg[0] += COMPENSATING_FACTOR * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }

    for(int t=0; t < N - 2; t++) {
      fg[0] += STEERING_ANGLE_DELTA_FACTOR * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += ACCELERATION_DELTA_FACTOR * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for(int t = 1; t < N; t++) {
      //The state at time t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      //The state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Taking into account Latency (i.e., use previous actuations)
      // Car comes to phantom stop after some time if this latency check is absent
      if (t > 1) {
        AD<double> prevSteeringAngle = vars[delta_start + t - 2];
        AD<double> prevThrottle = vars[a_start + t - 2];

        delta0 = prevSteeringAngle;
        a0 = prevThrottle;
      }

      // 1st order polynomial
      // AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      // 2nd order polynomial
      // AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2);
      //3rd order polynomial
      AD<double> FIRST_ORDER_X  = coeffs[1] * x0;
      AD<double> SECOND_ORDER_X = coeffs[2] * CppAD::pow(x0,2);
      AD<double> THIRD_ORDER_X  = coeffs[3] * CppAD::pow(x0,3);

      AD<double> FIRST_ORDER_DESIRED_PSI  = coeffs[1];
      AD<double> SECOND_ORDER_DESIRED_PSI = 2 * coeffs[2] * x0;
      AD<double> THIRD_ORDER_DESIRED_PSI  = 3 * coeffs[3] * CppAD::pow(x0,2);

      AD<double> f0 = coeffs[0] + FIRST_ORDER_X + SECOND_ORDER_X + THIRD_ORDER_X;
      AD<double> psides0 = CppAD::atan(FIRST_ORDER_DESIRED_PSI + SECOND_ORDER_DESIRED_PSI + THIRD_ORDER_DESIRED_PSI);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

      // Setup the rest of the model constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  /*
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
  */
  vector<double> actuatorValues;
  actuatorValues.push_back(solution.x[delta_start]);
  actuatorValues.push_back(solution.x[a_start]);

  for (int actVal = 0; actVal < N - 1; actVal++) {
    actuatorValues.push_back(solution.x[x_start + actVal + 1]);
    actuatorValues.push_back(solution.x[y_start + actVal + 1]);
  }
  return actuatorValues;
}
