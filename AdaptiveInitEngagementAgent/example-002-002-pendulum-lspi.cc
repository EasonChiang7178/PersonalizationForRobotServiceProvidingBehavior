/*   This file is part of rl-lib
 *
 *   Copyright (C) 2010,  Supelec
 *
 *   Author : Herve Frezza-Buet and Matthieu Geist
 *
 *   Contributor :
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public
 *   License (GPL) as published by the Free Software Foundation; either
 *   version 3 of the License, or any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *   Contact : Herve.Frezza-Buet@supelec.fr Matthieu.Geist@supelec.fr
 *
 */

/*
  This example shows how to use a parametric representation of the
  Q-function and apply LSPI. The inverted pendulum problem is solved
  here.
*/

#include <rl.h>
#include <iostream>
#include <iomanip>
#include <gsl/gsl_vector.h>
#include <cmath>
#include <unistd.h>
#include <fstream>

// This is our simulator.
typedef rl::problem::inverted_pendulum::Simulator<rl::problem::inverted_pendulum::DefaultParam> Simulator;

// Definition of Reward, S, A, Transition and TransitionSet.
#include "example-defs-transition.h"

// Defines ActionIterator, Features and a RBF architecture.
#include "example-defs-pendulum-architecture.h"


// In order to evaluate the current policy, we use LSTD
class CustomParam {
public: 
  double gamma(void)   const {return .95;}
  double epsilon(void) const {return 0.2;}
};
typedef rl::default_param::LSTD<CustomParam> Param;

#define NB_OF_EPISODES          500
#define NB_ITERATION_STEPS       10
#define MAX_EPISODE_LENGTH     3000
#define NB_LENGTH_SAMPLES        20

#include "example-defs-test-iteration.h"


int main(int argc, char* argv[]) {

  int             episode,step,episode_length;
  std::ofstream   ofile;

  Simulator       simulator;
  TransitionSet   transitions;
  ActionIterator  a_iter;
  ActionIterator  a_toss;
  RBFFeature      phi;
  Param           param;
  
  auto architecture  = rl::architecture::linear::sa::dot_product(phi);
  auto critic        = rl::sa::lstd(architecture,param);
  auto argmax_critic = rl::sa::argmax(critic,a_iter);
  auto random_agent  = rl::agent::random(S(),a_toss);
  auto greedy_agent  = rl::agent::greedy(argmax_critic);

  try {

    // Let us initialize the random seed.
    rl::random::seed(getpid());

    // Let us fill a set of transitions from successive episodes,
    // using a random policy.
    for(episode=0;episode<NB_OF_EPISODES;++episode) {
      simulator.setPhase(Simulator::phase_type());
      rl::episode::sa::run_and_collect(simulator,random_agent,transitions,
				       MAX_EPISODE_LENGTH,episode_length);
    }

    // Let us try the random policy
    test_iteration(random_agent,0);
    
    // Now, let us improve the policy and measure its performance at each step.
    for(step=1;step <= NB_ITERATION_STEPS;++step) {
      rl::sa::batch_policy_iteration_step(argmax_critic,
					  transitions.begin(),transitions.end());
      test_iteration(greedy_agent,step);
    }

    // Now, we can save the q_theta parameter
    std::cout << "Writing lspi.data" << std::endl;
    ofile.open("lspi.data");
    if(!ofile)
      std::cerr << "cannot open file for writing" << std::endl;
    else {
      ofile << argmax_critic.parameter();
      ofile.close();
    }
  
  }
  catch(rl::exception::Any& e) {
    std::cerr << "Exception caught : " << e.what() << std::endl;
  }
  
  return 0;
}
