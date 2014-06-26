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

// Using the rl library implies having a simulator at your
// disposal. This what the agent has to interact with. Your simulator
// has to fit the rl::concept::Simulator. It means that it must define
// the same type and the same methods than rl::concept::Simulator. The
// classes defined within the namespace rl::concept play the role of a
// chart that you have to be compliant with.

// Let us define our own simulator. It consists in a 6 letter word,
// whose letters can only be B,O and N. An action consists of pushing
// one of those letters on the left of the word, thus popping away the
// rightmost letter. Reward of 1 is obtained when the word
// "BONOBO" is made. The simulation stops, with a very bad reward, if the word
// is a palindrom. This is quite a silly problem, but let us define
// it.

class Bonobo {
public:

  // The following typedefs are required by the rl::concept::Simulator.

  typedef std::string phase_type;       // The state of the simulator.
  typedef phase_type  observation_type; // What the agent perceives. Here, the agent sees the actual phase.
  typedef char        action_type;      // Action is 'B', 'O' or 'N'. We will use an exception to check that.
  typedef double      reward_type;      // Reward is classically a scalar.

private:

  // Here is our internal cuisine.
  std::string word;
  double r;

public:

  // This is for debugging
  bool verbose;

  // The following methods are also required by the rl::concept::Simulator.
  Bonobo(void) 
    : word("BONBON"), r(0), verbose(false)   {}
  ~Bonobo (void)                             {}
  void setPhase(const phase_type &s)         {word = s;}
  const observation_type& sense(void) const  {return word;}
  reward_type reward(void) const             {return r;}

  // Let us define an exception if bad letters are provided.
  class BadLetter : public rl::exception::Any {
  public:
    BadLetter(char letter, std::string comment) 
      : rl::exception::Any(std::string("Bad letter '")
			   + std::string(1,letter)
			   + std::string("' received : ")
			   + comment) {} 
  };

  

  // This is where things happen. Usually, this method is the most
  // difficult thing to write when using the library, since it
  // expresses the problem itself.
  void timeStep (const action_type &a) {
    bool terminated;

    // Let us check the letter
    switch(a) {
    case 'B':
    case 'O':
    case 'N':
      break;
    default:
      throw BadLetter(a,"in Bonobo::timeStep");
      break;
    }

    // ok, now let us push the letter in front of the word.
    std::string tmp = std::string(1,a)+word;
    word = tmp.substr(0,6);

    // Let us check if the simulation is finished (i.e if word is a
    // palindrom). If so, we have to raise the appropriate exception,
    // expected by the rllib algorithms.
    terminated = 
      word[0] == word[5]
      &&  word[1] == word[4]
      &&  word[2] == word[3];

    // Let us compute the reward.
    if(terminated)
      r = -100;
    else if(word == "BONOBO")
      r = 1;
    else
      r = 0;

    if(verbose)
      std::cout << word << " : " << r << std::endl;

    if(terminated)
      throw rl::exception::Terminal(std::string("Word : ") + word);
  }
};

// Ok, now let us rename the types with usual names.

typedef Bonobo                      Simulator;
typedef Simulator::observation_type S;
typedef Simulator::action_type      A;
typedef Simulator::reward_type      Reward;

// As action is a discrete set, it is sometimes usefull to iterate on
// them. This is what the rl::concept::Iterator is designed for. It is
// also usefull to toss an action randomly. This is what the
// rl::concept::Toss expresses. Let is define both an action iterator
// and toss in the same class, as follows.
class ActionIterator {
private:

  void check(A current, std::string comment) const {
    switch(current) {
    case 'B':
    case 'O':
    case 'N':
      break;
    default:
      throw Bonobo::BadLetter(current,comment);
      break;
    }
  }

public:

  typedef A value_type;

  int size(void) const {return 3;}

  A value(int rank) const {
    A action;
    switch(rank) {
    case 0: action = 'B'; break;
    case 1: action = 'O'; break;
    case 2: action = 'N'; break;
    default:
      throw rl::exception::BadIteratorRank(size(),rank," in ActionIterator::value().");
      break;
    }
    return action;
  }

  A first(void) const {
    return 'B';
  }
  
  bool hasNext(A current) const {
    check(current,"in ActionIterator::hasNext");
    return current != 'N';
  }

  A next(A current) const {
    A next_action;
    check(current,"in ActionIterator::next");
    
    switch(current) {
    case 'B': next_action = 'O'; break;
    case 'O': next_action = 'N'; break;
    case 'N':
    default :
      // We should have rather raised an exception here.
      next_action = 'B';
      break;
    }
    return next_action;
  }

  A random(void) const {
    A random_action = '*';
    switch((int)(rl::random::toss(0,size()))) {
    case 0: random_action = 'B'; break;
    case 1: random_action = 'O'; break;
    case 2: random_action = 'N'; break;
    }
    return random_action;
  }
};

// Note that such toss/iterators can easily be written if your
// discrete set has contiguous values. If available actions would have
// been 'D','E','F','G','H', then the whole thing would have been
// reduced by the follwing class.
typedef rl::Iterator<A,'D','H'> AnotherActionIterator;

// This is how an iterator can be used. You may never need this, since
// this is called within rl templates.
void test_action_iterator(void) {
  A a;
  ActionIterator         aiter;
  AnotherActionIterator  another_aiter;

  std::cout << "ActionIterator :";
  a = aiter.first();
  std::cout << " " << a;
  while(aiter.hasNext(a)) {
    a = aiter.next(a);
    std::cout << " " << a;
  }
  std::cout << std::endl;

  std::cout << "AnotherActionIterator :";
  a = another_aiter.first();
  std::cout << " " << a;
  while(another_aiter.hasNext(a)) {
    a = another_aiter.next(a);
    std::cout << " " << a;
  }
  std::cout << std::endl;

}

// This function shows how to run an episode with the types we have
// defined so far.
void run_episode_version_01(void) {
  Simulator      simulator;
  ActionIterator aiter;
  Reward         sum = 0;

  // Now, let us define an agent itself. If fits rl::concept::Agent,
  // providing a "policy" method that tells what to do when observing a
  // given state. Here, we define a random agent.
  auto agent = rl::agent::random(S(),aiter);

  std::cout << std::endl
	    << "Version 01" << std::endl
	    << "----------" << std::endl
	    << std::endl;

  simulator.verbose = true;
  simulator.setPhase("BONBON");
  try {
    while(true) {
      simulator.timeStep(agent.policy(simulator.sense()));
      sum += simulator.reward();
    }
  }
  catch(rl::exception::Terminal& e) {
    sum += simulator.reward();
    std::cout << "Terminated : " << e.what() << std::endl;
  }
  
  std::cout << "Total reward during episode : " << sum << std::endl;
}

// Now, let us introduce a major concept in rl, that is the
// transition. Indeed, each time the agent interacts with the
// simulator, the s,a,r,s' values are involved. In the rl library,
// such transition is rather a s,a,r,s',a', with a' a dummy value that
// can be set explicitly afterwards. Let us run an episode while
// collecting interactions in a transition... 
typedef rl::sa::Transition<S,A,Reward>      Transition;

void run_episode_version_02(void) {
  Simulator      simulator;
  ActionIterator aiter;
  auto           agent      = rl::agent::random(S(),aiter);
  Reward         sum        = 0;
  Transition     transition;

  std::cout << std::endl
	    << "Version 02" << std::endl
	    << "----------" << std::endl
	    << std::endl;

  simulator.verbose = true;
  simulator.setPhase("BBBOOO");
  do {
    rl::episode::sa::interaction(simulator,agent,transition);
    sum += transition.reward();
  } while(!transition.isTerminal()); 
  std::cout << "Total reward during episode : " << sum << std::endl;
}

// ... or directly in a transition set.
void run_episode_version_03(void) {
  Simulator               simulator;
  ActionIterator          aiter;
  auto                    agent  = rl::agent::random(S(),aiter);
  Reward                  sum    = 0;
  int                     episode_length;

  std::vector<Transition>           transition_set;
  std::vector<Transition>::iterator iter, iter_end;

  std::cout << std::endl
	    << "Version 03" << std::endl
	    << "----------" << std::endl
	    << std::endl;

  simulator.verbose = true;
  simulator.setPhase("BBBOOO");
  rl::episode::sa::run_and_collect(simulator,agent,transition_set,
				   0,episode_length);

  // Let us read the collected transitions
  for(iter = transition_set.begin(), iter_end = transition_set.end(); 
      iter != transition_set.end(); 
      ++iter)
    sum += (*iter).reward(); // *iter is a transition.
  std::cout << "Total reward during episode : " << sum << std::endl;
}


int main(int argc, char* argv[]) {
  test_action_iterator();
  run_episode_version_01();
  run_episode_version_02();
  run_episode_version_03();
  return 0;
}

