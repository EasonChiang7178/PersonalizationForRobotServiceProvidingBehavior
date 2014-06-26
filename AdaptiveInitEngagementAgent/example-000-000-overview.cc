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



// This example is an overview that gives you the taste of the
// library. You may have to read the next examples to have a better
// understanding of the details. It implements sarsa for the
// cliff-walking problem.

#include <iostream>
// This is the rl header
#include <rl.h>

// These are useful typedefs

typedef rl::problem::cliff_walking::Cliff<20,6>              Cliff;      // World size.
typedef rl::problem::cliff_walking::Simulator<Cliff>         Simulator;  // This is the dynamical system to control.
typedef Simulator::reward_type                               Reward; 
typedef Simulator::observation_type                          S;
typedef Simulator::action_type                               A;
typedef rl::sa::Transition<S,A,Reward>                       Transition; // (s,a,r,s',a') tuple.

// As action space is finite, this class allows internal algorithms to
// iterate on each possible action value. Actions values are declared
// as consecutive actionNorth, actionSouth, actionEast and actionWest
// integers in rl-cliff-walking.h, they are thus in the range from
// actionNorth to actionWest. These range bounds appear in the
// follwing definition.
typedef rl::Iterator<A,
		     rl::problem::cliff_walking::actionNorth, 
		     rl::problem::cliff_walking::actionWest> ActionIterator;

// This is a parameter class
class Param {
public: 
  double gamma(void)   const {return .99;}
  double alpha(void)   const {return .05;}
  double epsilon(void) const {return 0.2;}
};

// The Q-function is tabular, i.e. the Q(s,a) values are stored in a
// vector. As the rllib is oriented toward function approximation for
// Q functions, dealing with some tabular representation requires an
// encapsulation of the table, since a tabular representation is a
// specific case of a more general function representation.
#define S_CARDINALITY         Cliff::size
#define A_CARDINALITY         rl::problem::cliff_walking::actionSize
#define TABULAR_Q_CARDINALITY S_CARDINALITY*A_CARDINALITY  // Array size for storing the Q[s,a].
#define TABULAR_Q_RANK(s,a)   a*S_CARDINALITY+s            // Index of the Q[s,a] value in the monodimentional array.

class TabularQ {
public:
  typedef S 	      state_type;
  typedef A           action_type;
  typedef gsl_vector* param_type;
  typedef rl::SA<S,A> input_type;
  typedef Reward      output_type;

  param_type newParameterInstance(void) const {
    // The gsl_vector, i.e. the theta parameter, actually stores each
    // Q[s,a]. It is the array of values that represents Q.
    return gsl_vector_calloc(TABULAR_Q_CARDINALITY);
  }
  output_type operator()(const param_type theta, state_type s, action_type a) const {
    // This returns Q[s,a] from the vector theta.
    return gsl_vector_get(theta,TABULAR_Q_RANK(s,a));
  }
  void gradient(const param_type theta, state_type s,action_type a, param_type grad) const {
    gsl_vector_set_basis(grad,TABULAR_Q_RANK(s,a)); // 0000010000000 with 1 at [s,a]
  }

  // These are the same methods but with different argument
  // types. They just call previously defined methods. They are
  // required for compliance with internal computation.
  output_type operator()(const param_type theta, const input_type &sa) const {
    return (*this)(theta,sa.s,sa.a);
  }
  void gradient(const param_type theta, const input_type& sa, param_type grad) const {
    gradient(theta,sa.s,sa.a,grad);
  }
};

// Let us start some experiment
int main(int argc, char* argv[]) {
  Simulator                simulator;                
  Transition               transition;               
  ActionIterator           action_iter;
  Param                    param;
  TabularQ                 q;
  
  auto                     critic        = rl::sa::sarsa(q,param);
  auto                     argmax_critic = rl::sa::argmax(critic,action_iter);   
  auto                     explore_agent = rl::agent::epsilon_greedy(argmax_critic,action_iter,param); 
  auto                     test_agent    = rl::agent::greedy(argmax_critic);   
  A                        a;                        
  S                        s;                        

  int                      episode,actual_episode_length,step;

  // Let us run 10000 episodes with the agent that learns the Q-values.
  for(episode = 0; episode < 10000; ++episode) {
    simulator.restart();
    rl::episode::sa::run_and_learn(simulator,explore_agent,critic,transition,0,actual_episode_length);
  }

  step = 0;

  // Let us be greedy on the policy we have found, using the greedy
  // agent to run an episode.
  try {
    simulator.restart();
    while(true) {
      step++;
      s = simulator.sense();
      a = test_agent.policy(s);
      simulator.timeStep(a);
    }
  }
  catch(rl::exception::Terminal e) {
    std::cout << "Best policy episode ended after " << step << " steps." << std::endl;
  }
  
  
  return 0;
}

