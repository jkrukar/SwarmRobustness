#ifndef BROWNIAN_LOOP_FUNCTIONS_H
#define BROWNIAN_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CBrownianLoopFunctions : public CLoopFunctions {

public:

  CBrownianLoopFunctions(); 
  virtual ~CBrownianLoopFunctions() {}
  virtual void Destroy(); 
  virtual void Reset();
  virtual void Init(TConfigurationNode& t_tree);
  virtual void PreStep();
  virtual void PostStep();

private:

  CRandom::CRNG* c_fRNG;
  CRange<UInt32> c_failure_range;
  CRange<UInt32> c_failure_time;
  UInt32 reliability_N; // Number of Epucks in simulation 10, 30, 50, 70, 100, 120
  UInt32 reliability_k; // 90% of N for k-out-of-N:G system (10% fail)
  UInt32 r_failure_case; // Current experiment failure case 1, 2, 3
  UInt32 s_run; // Current simulation number 1 - 100
  UInt32 s_time; // Total run time of simulation
  UInt32 s_num_to_complete;  // Number of Epucks required to reach beacon to terminate (50% within 200cm)
  Real s_distance; // Total distance traveled by swarm (measured by center of swarm)
  CVector2 s_previous_center; // Previous center of swarm position
  std::string str_output;
  std::ofstream c_output;

  void setFailureCasesInSwarm();
};

#endif
