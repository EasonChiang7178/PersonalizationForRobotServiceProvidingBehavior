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
#include <iostream>
#include <string>
#include <vector>

// Here, the purpose is to introduce the concepts that are related to
// learning. Let us start by defining a problem to solve. Let us
// consider, once again, a silly problem. The state is a scalar value
// taken in [0,1]. There are 3 action, names 'raise', 'lower' and
// 'none', that add or remove .05 to x. The system gets rewarded when
// the value reaches 1... that is also the terminal state.
class Simulator {
public:
  typedef double                    phase_type;       
  typedef phase_type                observation_type; 
  typedef enum Action {actionLower,
		       actionNone,
		       actionRaise} action_type; 
  typedef double                    reward_type;  
  
private:
  phase_type x;
  reward_type r;

public:

  class SomethingWrong :  public rl::exception::Any {
  public: 
    SomethingWrong(std::string comment) 
      : rl::exception::Any(std::string("Something went wrong : ")
			   + comment) {} 
  };

  Simulator(void)
    : x(.5), r(0) {}
  ~Simulator(void) {}

  void setPhase(const phase_type &s)         {x = s;}
  const observation_type& sense(void) const  {return x;}
  reward_type reward(void) const             {return r;}
  
  void timeStep (const action_type &a) {
    double dx;

    switch(a) {
    case actionLower: x-=.05; break;
    case actionNone:          break;
    case actionRaise: x+=.05; break;
    default:
      throw SomethingWrong("Simulator::timeStep : Bad action.");
    }
    
    r = 0;
    x += dx;
    if(x<0)
      x = 0;
    else if(x>=1) {
      x=1;
      r=1;
      throw rl::exception::Terminal("1 is reached");
    }
  }
};

// Ok, now let us rename the types with usual names.
typedef Simulator::observation_type S;
typedef Simulator::action_type      A;
typedef rl::SA<S,A>                 SA;
typedef Simulator::reward_type      Reward;

// We can also define an action iterator and toss (we will not use it
// here).
rl::Iterator<A,
	     Simulator::actionLower,
	     Simulator::actionRaise> ActionIterator;

// And we can define what transitions are.
typedef rl::sa::Transition<S,A,Reward>  Transition;
typedef std::vector<Transition>         TransitionSet;

// Let us now start with the definition of function-related concepts,
// that are used in the rl-library for learning.

// For linear algorithms, it is sometimes required by the rl library
// to implement a feature space. It consists in representing a (s,a)
// pair by a vector. Let us implement rl::concept::sa::Feature for
// that, using gaussian radial basis functions.
class Feature {
public:

  typedef SA 	sa_type;
  typedef SA    input_type;
  typedef S 	state_type;
  typedef A 	action_type;

  enum {dimension=9};
  
  void operator()(const input_type &x,
		  gsl_vector *phi) const {
    (*this)(x.s,x.a,phi);
  }
  
  void operator()(state_type s, action_type a, 
		  gsl_vector *phi) const {
    int offset;
    double dist;

    if(phi->size != 9)
      throw Simulator::SomethingWrong("Feature::operator() : Bad phi size");
    
    switch(a) {
    case Simulator::actionLower: offset = 0; break;
    case Simulator::actionNone:  offset = 3; break;
    case Simulator::actionRaise: offset = 6; break;
    default:
      throw Simulator::SomethingWrong("Feature::operator()  : Bad action.");
    }

    gsl_vector_set_zero(phi);
    dist = s;    gsl_vector_set(phi,offset,  exp(-20*dist*dist));
    dist = s-.5; gsl_vector_set(phi,offset+1,exp(-20*dist*dist));
    dist = s-1;  gsl_vector_set(phi,offset+2,exp(-20*dist*dist));
  }
};

// Some other algorithms use parametrized functions, to approximate
// Q-values for example. This is what a regression is. The
// parametrization scheme is called an architecture in the rl
// library. Providing a vector of parameters to an architecture
// defines an actual function. Let us define a very simple linear
// architecture here, for representing Q values. It fits
// rl::sa::Architecture.
class Architecture {
public:
  typedef SA                 sa_type;
  typedef S                  state_type;
  typedef A                  action_type;
  typedef sa_type            input_type;
  typedef Reward             output_type;
  typedef gsl_vector*        param_type;
  typedef Transition         transition_type;
  typedef transition_type    sa_transition_type;

  Architecture(void) {}

  output_type operator()(const param_type &theta, 
			 const input_type &x) const {
    return (*this)(theta,x.s,x.a);
  }
  
  param_type newParameterInstance (void) const {
    return gsl_vector_calloc(6);
  }

  output_type operator()(const param_type &theta, 
			 const state_type &s, 
			 const action_type &act) const {
    double a,b;
    int offset;

    if(theta->size != 6)
      throw Simulator::SomethingWrong("Architecture::operator() : Bad theta size");
    
    switch(act) {
    case Simulator::actionLower: offset = 0; break;
    case Simulator::actionNone:  offset = 2; break;
    case Simulator::actionRaise: offset = 4; break;
    default:
      throw Simulator::SomethingWrong("Architecture::operator()  : Bad action.");
    }
    
    a = gsl_vector_get(theta,offset);
    b = gsl_vector_get(theta,offset+1);

    return a*s+b;
  }
};

// This main just aims at instanciating templates in order to check
// they are correct.
int main(int argc, char* argv[]) {
  Simulator s;
  Architecture a;
  Feature f;

  std::cout << "That's it." << std::endl;
}

				
