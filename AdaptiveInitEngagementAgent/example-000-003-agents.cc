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


// First, you have to include the rllib header file, and the other
// usual includes that you may need.
#include <rl.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

// In the rl library, there are ready to use templates for building
// agents. An agent is something that knows what to do when it is
// given a state. In other words, it implements a policy.

// We will consider here agents built from an available Q-Value
// representation. This Q-Value is the actual critic. Here, we will
// consider a n-armed bandit problem, where the Q value are related
// tgo a single state. The critic is then only an evaluation of each
// arm (the actions are "play with arm #i"). For this example, It is enough to have a rl::concept::sa::function for implementing the Q-values.

#define NB_ARMS 50

typedef int                         S; // a dummy state here
typedef int                         A; // An action is the arm number (from 0)
typedef double                      Reward;
typedef rl::Iterator<A,0,NB_ARMS-1> ActionIterator;
typedef ActionIterator              ActionToss;      // rl::Iterator is a two-in-one type.

class Param {
public: 
  double t;
  double temperature(void) const {return t;}
  double epsilon(void) const {return .5;}
};

// The critic here have to fit rl::concept::sa::Function
class Q {
public:
  typedef rl::SA<S,A> input_type;
  typedef Reward      output_type;
  typedef S           state_type;
  typedef A           action_type;

  // Here are stored our Q values
  double data[NB_ARMS];
  
  output_type operator()(const input_type& x) const {
    return (*this)(x.s,x.a);
  }

  output_type operator()(const state_type& s, 
			 const action_type& a) const {
    return data[a];
  }
};


// Let us define functions for plotting histograms

#define HISTO_NB_SAMPLES 20000

template<typename AGENT>
void plot1D(std::string title,const AGENT& agent,std::string filename) {
  std::ofstream file;

  file.open(filename.c_str());
  if(!file) {
    std::cerr << "Cannot open \"" << file << "\". Aborting";
    return;
  }

  int histogram[NB_ARMS];
  A a;
  S dummy;
  int i;
  int max=0;

  for(a = 0; a < NB_ARMS; ++a)
    histogram[a] = 0;
  for(i = 0; i < HISTO_NB_SAMPLES; ++i)
    histogram[agent.policy(dummy)]++;
  for(a = 0; a < NB_ARMS; ++a)
    if(histogram[a] > max)
      max = histogram[a];
  file << "set title '" << title << "';" << std::endl
       << "set xrange [0:" << NB_ARMS-1 << "];" << std::endl
       << "set yrange [0:" << max*1.1 << "];" << std::endl
       << "set xlabel 'Actions'" << std::endl
       << "plot '-' with lines notitle" << std::endl;
  for(a = 0; a < NB_ARMS; ++a)
    file << a << ' ' << histogram[a] << std::endl;

  file.close();
  std::cout << "\"" << filename << "\" generated." << std::endl;
}

template<typename AGENT>
void plot2D(Param& param,const AGENT& agent) {
  std::ofstream file;

  file.open("SoftMaxAgent.plot");
  if(!file) {
    std::cerr << "Cannot open \"SoftMaxAgent.plot\". Aborting";
    return;
  }

  int histogram[NB_ARMS];
  A a;
  S dummy;
  int i;
  int tpt;

  file << "set title 'SoftMax agent action choices';" << std::endl
       << "set xrange [0:" << NB_ARMS-1 << "];" << std::endl
       << "set xlabel 'Temperature'" << std::endl
       << "set ylabel 'Actions'" << std::endl
       << "set hidden3d;" << std::endl
       << "set ticslevel 0;" << std::endl
       << "splot '-' using 1:2:3 with lines notitle" << std::endl;
  for(tpt = 0, param.t = 100; 
      tpt < 50; 
      file << std::endl, param.t *= .85, ++tpt) {
    for(a = 0; a < NB_ARMS; ++a)
      histogram[a] = 0;
    for(i = 0; i < HISTO_NB_SAMPLES; ++i)
      histogram[agent.policy(dummy)]++;
    for(a = 0; a < NB_ARMS; ++a)
      file << tpt << ' ' << a << ' ' << histogram[a]/(double)HISTO_NB_SAMPLES << std::endl;
    std::cout << "line " << std::setw(3) << tpt+1 << "/50 generated.   \r" << std::flush;
  }

  file.close();
  std::cout << "\"SoftMaxAgent.plot\" generated.                 " << std::endl;
}

int main(int argc, char* argv[]) {
  Q              q;
  ActionIterator a_iter;
  ActionToss     a_toss;
  A              a;
  double         x;
  Param          param;

  auto argmax_q             = rl::sa::argmax(q,a_iter);
  auto random_agent         = rl::agent::random(S(),a_toss);
  auto greedy_agent         = rl::agent::greedy(argmax_q);
  auto epsilon_greedy_agent = rl::agent::epsilon_greedy(argmax_q,a_toss,param);
  auto softmax_agent        = rl::agent::softmax(q,a_iter,param);

  try {
    // Let us initialize the values with a bi-modal distribution...
    // I have found it empirically while playing with gnuplot.
    for(a = 0; a < NB_ARMS; ++a) {
      x = a/(double)NB_ARMS;
      q.data[a] = (1-.2*x)*pow(sin(5*(x+.15)),2);
    }

    // Let us plot histograms of agents.
    plot1D("Random agent choices",random_agent,"RandomAgent.plot");
    plot1D("Greedy agent choices",greedy_agent,"GreedyAgent.plot");
    plot1D("Epsilon-greedy agent choices",epsilon_greedy_agent,"EpsilonGreedyAgent.plot");

    plot2D(param,softmax_agent);
  }
  catch(rl::exception::Any& e) {
    std::cerr << e.what() << std::endl;
  }
  return 0;
}
