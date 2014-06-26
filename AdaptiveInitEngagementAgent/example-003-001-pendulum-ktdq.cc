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
#include <unistd.h>

// We define our own parameters for the inverted pendulum
class ipParams{
public:
  // This is the amplitude of the noise (relative) applied to the action.
  inline static double actionNoise(void)        {return  0.2;}
  // This is the noise of angle perturbation from the equilibrium state at initialization.
  inline static double angleInitNoise(void)     {return 1e-3;}
  // This is the noise of speed perturbation from the equilibrium state at initialization.
  inline static double speedInitNoise(void)     {return 1e-3;}
  
};

// This is our simulator.
typedef rl::problem::inverted_pendulum::Simulator<ipParams> Simulator;

// Definition of Reward, S, A, Transition and TransitionSet.
#include "example-defs-transition.h"

// Defines ActionIterator, Features and a RBF architecture.
#include "example-defs-pendulum-architecture.h"

class CustomParam {
public: 
  double gamma(void)   const {return .95;}
  double epsilon(void) const {return 0.2;}
}; 

// As the architecture is linear, we can accelerate evaluation.
class Param : public rl::default_param::KTD<CustomParam> {
public:
  bool use_linear_evaluation(void) const {return true;}
};

#define NB_OF_EPISODES         1000
#define NB_LENGTH_SAMPLES         5
#define MAX_EPISODE_LENGTH     3000
#define TEST_PERIOD             100

#include "example-defs-test-iteration.h"
#include "example-defs-ktdq-experiments.h"

int main(int argc, char* argv[]) {

  RBFFeature      phi;
  ActionIterator  a_iter;
  Param           param;
  
  auto architecture  = rl::architecture::linear::sa::dot_product(phi);
  auto greedy        = rl::sa::greedy(a_iter); 
  auto argmax_critic = rl::sa::ktd_q(architecture,greedy,a_iter,param);

  make_experiment(argmax_critic);

  return 0;
}
