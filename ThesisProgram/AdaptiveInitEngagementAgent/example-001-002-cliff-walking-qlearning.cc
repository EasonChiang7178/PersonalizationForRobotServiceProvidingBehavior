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


#include <rl.h>
#include <cstdlib>

typedef rl::problem::cliff_walking::Cliff<20,6> Cliff;
typedef rl::problem::cliff_walking::Simulator<Cliff> Simulator;

// Definition of Reward, S, A, SA, Transition and TransitionSet.
#include "example-defs-transition.h"

// Definition of ActionIterator, TabularQ
#include "example-defs-tabular-cliff.h"


class Param {
public: 
  double gamma(void)   const {return .99;}
  double alpha(void)   const {return .05;}
  double epsilon(void) const {return 0.2;}
};


#include "example-defs-cliff-experiments.h"

int main(int argc, char* argv[]) {
  TabularQ   q;
  Param      param;
  ActionIterator a_iter;

  auto greedy        = rl::sa::greedy(a_iter); 
  auto argmax_critic = rl::sa::q_learning(q,greedy,param);

  make_experiment(param,argmax_critic);
  return 0;
}
