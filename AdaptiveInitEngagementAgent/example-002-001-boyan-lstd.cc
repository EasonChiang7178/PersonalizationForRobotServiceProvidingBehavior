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
  This examples shows the application of LSTD to the Boyan chain.
*/

#include <rl.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

// This is our simulator.
typedef rl::problem::boyan_chain::Simulator Simulator;

// MDP features
typedef Simulator::reward_type            Reward; 
typedef Simulator::observation_type       S;
typedef Simulator::action_type            A;

// Let us define an agent, that chooses a random action... among the
// single one available !
typedef rl::Iterator<A,
		     rl::problem::boyan_chain::actionNone,
		     rl::problem::boyan_chain::actionNone> ActionIterator;

// Let us define transitions s,r,s'
typedef rl::Transition<S,Reward>          Transition;
typedef std::vector<Transition>           TransitionSet;

// The function that associates a feature vector to a State is the
// following.
typedef rl::problem::boyan_chain::Feature Feature;

// Let us now set up the parameters. It must fit the
// rl::concept::param::LSTD and rl::concept::param::TD.
class Param {
public: 
  double gamma(void) const {return 1;}
  double reg(void)   const {return 0;}
  double alpha(void) const {return 0.05;}
};

#define NB_OF_EPISODES 100
int main(int argc, char* argv[]) {

  Simulator         simulator;
  Transition        transition;
  TransitionSet     transitions;
  int               episode_length;
  Feature           phi;
  Param             param;
  ActionIterator    a_toss;

  int               episode;
  const gsl_vector* theta_lstd;
  gsl_vector*       theta_loaded;
  std::ofstream     ofile;
  std::ifstream     ifile;

  auto archi         = rl::architecture::linear::dot_product(phi);
  auto lstd_critic   = rl::lstd(archi,param);
  auto td_critic1    = rl::td(archi,param);
  auto td_critic2    = rl::td(archi,param);
  auto explore_agent = rl::agent::random(S(),a_toss);

  try {
    
    // Let us fill a set of transitions from successive episodes.
    for(episode = 0; episode < NB_OF_EPISODES; ++episode) {
      simulator.initPhase();
      rl::episode::run_and_collect(simulator,explore_agent,transitions,
				   0,episode_length);
    }

    // Now, we have to apply LSTD to the transition database.
    lstd_critic.update(transitions.begin(), transitions.end());
    theta_lstd = lstd_critic.parameter();

    // Now, we have to apply TD to the transition database.
    td_critic1.update(transitions.begin(), transitions.end());

    // But we could have learned really on-line (with td2)
    for(episode = 0; episode < NB_OF_EPISODES; ++episode) {
      simulator.initPhase();
      rl::episode::run_and_learn(simulator,explore_agent,td_critic2,transition,
				 0,episode_length);
    }

    // You can save with >> 
    std::cout << "Writing lstd.data" << std::endl;
    ofile.open("lstd.data");
    if(!ofile)
      std::cerr << "cannot open file for writing" << std::endl;
    else {
      ofile << theta_lstd;
      ofile.close();
    }

    // You can load back with <<
    std::cout << "Reading lstd.data" << std::endl;
    ifile.open("lstd.data");
    if(!ifile)
      std::cerr << "cannot open file for reading" << std::endl;
    else {
      // As theta_loaded is freed by >>, we have to set non previously allocated vectors to 0.
      theta_loaded = (gsl_vector*)0;
      ifile >> theta_loaded;
      ifile.close();
    }
      
    // Let us display the result
    std::cout << std::endl
	      << "LSTD estimation          : ("
	      << std::setw(15) << gsl_vector_get(theta_lstd,0) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,1) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,2) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,3) << ')'
	      << std::endl
	      << "LSTD estimation (loaded) : ("
	      << std::setw(15) << gsl_vector_get(theta_loaded,0) << ','
	      << std::setw(15) << gsl_vector_get(theta_loaded,1) << ','
	      << std::setw(15) << gsl_vector_get(theta_loaded,2) << ','
	      << std::setw(15) << gsl_vector_get(theta_loaded,3) << ')'
	      << std::endl;
    theta_lstd = td_critic1.parameter();
    std::cout << "TD offline estimation    : ("
	      << std::setw(15) << gsl_vector_get(theta_lstd,0) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,1) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,2) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,3) << ')'
	      << std::endl;
    theta_lstd = td_critic2.parameter();
    std::cout << "TD online estimation     : ("
	      << std::setw(15) << gsl_vector_get(theta_lstd,0) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,1) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,2) << ','
	      << std::setw(15) << gsl_vector_get(theta_lstd,3) << ')'
	      << std::endl;
    std::cout << "Optimal one should be    : ("
	      << std::setw(15) << -24  << ','
	      << std::setw(15) << -16  << ','
	      << std::setw(15) <<  -8  << ','
	      << std::setw(15) <<   0  << ')'
	      << std::endl;
  
    // Don't forget to free 
    gsl_vector_free(theta_loaded);
  }
  catch(rl::exception::Any& e) {
    std::cerr << "Exception caught : " << e.what() << std::endl;
  }
  
  return 0;
}
