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
  Q-function and apply KTD-Q. The inverted pendulum problem is solved
  here. It also provide the computed variance (useless here, but show how it can be obtained).
*/

#include <rl.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

// This is our simulator.
typedef rl::problem::mountain_car::DefaultParam        mcParam;
typedef rl::problem::mountain_car::Simulator<mcParam>  Simulator;


// Definition of Reward, S, A, Transition and TransitionSet.
#include "example-defs-transition.h"


// Defines ActionIterator (useless here), Features and a RBF architecture.
#include "example-defs-mountain-car-architecture.h"

// KTD definition
class Param {
public:  
  static double epsilon(void) {return 0.1;}
  static double gamma(void) {return .95;}
  static double eta_noise(void)              {return 1e-5;}
  static double observation_noise(void)      {return 1;}
  static double priorVar(void)               {return 10;}
  static double randomAmplitude(void)        {return 1e-1;}
  static double ut_alpha(void)               {return 1e-1;}
  static double ut_beta(void)                {return 2;}
  static double ut_kappa(void)               {return 0;}
  static bool   use_linear_evaluation(void)  {return true;} // We actually use a linear architecture.
};


typedef rl::problem::mountain_car::Gnuplot<Simulator>  Gnuplot;

#define MAX_EPISODE_LENGTH_LEARN          1500
#define MAX_EPISODE_LENGTH_TEST            300
#define KTDSARSA_FILENAME   "mountain-car.ktdsarsa"

void test(const Simulator::phase_type& start);
void train(int nb_episodes, bool make_movie);

int main(int argc, char* argv[]) {
  bool                  learn_mode;
  bool                  movie_mode=false;
  int                   nb_episodes;
  Simulator::phase_type init_phase;
  Simulator             simulator;
  std::string           arg;

  rl::random::seed(getpid());


  if(argc < 2) {
    std::cerr << "Usage : " << std::endl
	      << "  " << argv[0] << " learn <nb-episodes>   (100 episode should be enough)" << std::endl
	      << "  " << argv[0] << " learnandmovie <nb-episodes>   (100 episode should be enough)" << std::endl
	      << "  " << argv[0] << " test bottom" << std::endl
	      << "  " << argv[0] << " test random" << std::endl
	      << "  " << argv[0] << " test <position> <speed>" << std::endl;
    return 0;
  }


  arg = argv[1];

  if(arg == "learnandmovie")
    movie_mode=true;

  if(arg == "learn" || arg == "learnandmovie") {
    learn_mode = true;
    if(argc == 3)
      nb_episodes = atoi(argv[2]);
    else {
      std::cerr << "Bad command syntax. Aborting." << std::endl;
      return 1;
    }
  }
  else if(arg == "test") {
    learn_mode = false;
    if(argc == 3) {
      arg = argv[2];
      if(arg == "bottom")
	init_phase = Simulator::phase_type(simulator.bottom(),0);
      else if(arg == "random")
	init_phase = Simulator::phase_type();
      else {
	std::cerr << "Bad command syntax. Aborting." << std::endl;
	return 1;
      }
    }
    else if(argc==4)
      init_phase = Simulator::phase_type(atof(argv[2]),atof(argv[3]));
    else {
      std::cerr << "Bad command syntax. Aborting." << std::endl;
      return 1;
    }
  }
  else {
    std::cerr << "Set learning mode to test or learn. Aborting." << std::endl;
    return 1;
  }


  if(learn_mode)
    train(nb_episodes,movie_mode);
  else
    test(init_phase);
  return 0;
}

void train(int nb_episodes, bool make_movie) {
  int            episode,step,episode_length;
  std::string    command;
  std::ofstream  file;

  Simulator      simulator;
  Transition     transition;
  Param          param;
  ActionIterator a_iter;
  ActionIterator a_toss;
  RBFFeature     phi;

  auto architecture         = rl::architecture::linear::sa::dot_product(phi);
  auto argmax_critic        = rl::sa::ktd_sarsa(architecture,a_iter,param);
  auto explore_agent        = rl::agent::epsilon_greedy(argmax_critic,a_toss,param);
  auto greedy_agent         = rl::agent::greedy(argmax_critic);


  // Let us initialize the random seed.
  rl::random::seed(getpid());

  try {
    
    step = 0;
    for(episode = 0; episode < nb_episodes; ++episode) {

      std::cout << "Running episode " << episode+1 << "/" << nb_episodes << "." << std::endl;
      simulator.setPhase(Simulator::phase_type()); 
      rl::episode::sa::run_and_learn(simulator,explore_agent,argmax_critic,transition,
				     MAX_EPISODE_LENGTH_LEARN,episode_length);
      std::cout << "... length is " << episode_length << "." << std::endl;
      
      ++step;

      if(make_movie)
	Gnuplot::drawQ("KTD Sarsa + RBF",
		       "ktd",step,
		       argmax_critic,greedy_agent);
    }
	
    // Let us save the results.
    file.open(KTDSARSA_FILENAME);
    if(!file)
      std::cerr << "Cannot open \"" << KTDSARSA_FILENAME << "\"." << std::endl;
    else {
      file << std::setprecision(20) << argmax_critic;
      file.close();
    }

    if(make_movie) {

      std::string command;

      command = "find . -name \"ktd-*.plot\" -exec gnuplot \\{} \\;";
      std::cout << "Executing : " << command << std::endl;
      system(command.c_str());

      command = "find . -name \"ktd-*.png\" -exec convert \\{} -quality 100 \\{}.jpg \\;";
      std::cout << "Executing : " << command << std::endl;
      system(command.c_str());

      command = "ffmpeg -i ktd-%06d.png.jpg -b 1M rllib.avi";
      std::cout << "Executing : " << command << std::endl;
      system(command.c_str());

      command = "find . -name \"ktd-*.plot\" -exec rm \\{} \\;";
      std::cout << "Executing : " << command << std::endl;
      system(command.c_str());

      command = "find . -name \"ktd-*.png\" -exec rm \\{} \\;";
      std::cout << "Executing : " << command << std::endl;
      system(command.c_str());
    
      command = "find . -name \"ktd-*.png.jpg\" -exec rm \\{} \\;";
      std::cout << "Executing : " << command << std::endl;
      system(command.c_str());
    }
  }
  catch(rl::exception::Any& e) {
    std::cerr << "Exception caught : " << e.what() << std::endl; 
  }
}

void test(const Simulator::phase_type& start) {
  std::ifstream  file;

  Simulator      simulator;
  Param          param;
  ActionIterator a_iter;
  RBFFeature     phi;

  auto architecture         = rl::architecture::linear::sa::dot_product(phi);
  auto argmax_critic        = rl::sa::ktd_sarsa(architecture,a_iter,param);
  auto greedy_agent         = rl::agent::greedy(argmax_critic);

  try {
    
    file.open(KTDSARSA_FILENAME);
    if(!file) {
      std::cerr << "Cannot open \"" << KTDSARSA_FILENAME << "\"." << std::endl;
      ::exit(1);
    }
    
    // Let us load some critic ...
    file >> argmax_critic;

    // ... and run an episode.
    simulator.setPhase(start); 
    Gnuplot::drawEpisode("Mountain car run",
			 "mountain-car-run",-1,
			 simulator,argmax_critic,
			 greedy_agent,
			 MAX_EPISODE_LENGTH_TEST);
  }
  catch(rl::exception::Any& e) {
    std::cerr << "Exception caught : " << e.what() << std::endl; 
  }
}
