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

#ifndef rlTRANSITION_H
#define rlTRANSITION_H

#include <vector>
#include <rlTypes.h>
#include <rlConcept.h>

namespace rl {
  
  /**
   * @short This encodes a transition.
   *
   */
  template<typename INPUT,typename REWARD> 
  class Transition {

  public:

    typedef INPUT  zset_type;
    typedef REWARD reward_type;            

  protected:

    zset_type z;
    reward_type r;
    zset_type zz;
    bool is_terminal;
    
  public:

    Transition(void) 
      : z(), r(0), zz(), is_terminal(false) {}

    Transition(const zset_type& current,
	       reward_type reward) 
      : z(current),r(reward),zz(),is_terminal(true) {}
    Transition(const zset_type& current,
	       reward_type reward,
	       const zset_type& next) 
      : z(current),r(reward),zz(next),is_terminal(false) {}

    Transition(const Transition<zset_type,reward_type>& t)
      : z(t.z), r(t.r), zz(t.zz), is_terminal(t.is_terminal) {}

    Transition<zset_type,reward_type>& operator=(const Transition<zset_type,reward_type>& t) {
      if(this != &t) {
	z           = t.z;
	r           = t.r;
	zz          = t.zz;
	is_terminal = t.is_terminal;
      }
      return *this;
    }

    /**
     * @return The initial state of the transition.
     */
    const zset_type& current(void) const {return z;} 

    /**
     * @return The initial next state of the transition. This may throw rl::exception::Terminal.
     */
    const zset_type& next(void) const {
      if(is_terminal)
	throw rl::exception::Terminal("occurred in Transition::next");
      else
	return zz;
    }

    /**
     * Tells wether the transition leads to a terminal state.
     */
    bool isTerminal(void) const {return is_terminal;}

    /**
     * @return the reward obtained from that transition.
     */
    reward_type reward(void) const {return r;} 
  };

  namespace sa {
    /**
     * @short SA transition.
     * 
     */
    template<typename STATE,typename ACTION,typename REWARD> 
    class Transition 
      : public rl::Transition< rl::SA<STATE, ACTION>, REWARD > {
    public:
      
      typedef rl::SA<STATE, ACTION> zset_type;
      typedef REWARD                reward_type;   
      typedef STATE                 state_type;
      typedef ACTION                action_type;
      
      Transition(void) 
	: rl::Transition<zset_type,reward_type> () {}
      Transition(const Transition<state_type,action_type,reward_type>& t) 
	: rl::Transition<zset_type,reward_type> (t) {}

      Transition(const state_type& s,
		 const action_type& a,
		 reward_type reward,
		 const state_type& next_s,
		 const action_type& next_a)
	: rl::Transition<zset_type,reward_type>(rl::sa_pair(s,a),
						reward,
						rl::sa_pair(next_s,next_a)) {}
      Transition(const state_type s,
		 const action_type a,
		 reward_type reward)
	: rl::Transition<zset_type,reward_type>(rl::sa_pair(s,a),reward) {}
      
      void setNextAction(const action_type& a) {
	this->zz.a = a;
      }

      const state_type& currentState(void) const {
	return this->z.s;
      }

      const state_type& nextState(void) const {
	return this->zz.s;
      }

      const action_type& currentAction(void) const {
	return this->z.a;
      }

      const action_type& nextAction(void) const {
	return this->zz.a;
      }
      
    };
  }

  namespace episode {
    /**
     * This triggers an interaction from an action and fill a transition.
     */
    template<typename SIMULATOR,typename ACTION,typename TRANSITION>
    void perform(SIMULATOR& simulator,
		 ACTION action,
		 TRANSITION& transition) {
      auto current = simulator.sense();
      try {
	simulator.timeStep(action);
	transition = TRANSITION(current,
				simulator.reward(),
				simulator.sense());
      }
      catch(rl::exception::Terminal& e) {
	transition = TRANSITION(current,simulator.reward());
      }
    }

    /**
     * This triggers an interaction from an agent and fill a transition.
     */
    template<typename SIMULATOR,typename AGENT,typename TRANSITION>
    void interaction(SIMULATOR& simulator,
		     const AGENT& agent,
		     TRANSITION& transition) {
      auto current = simulator.sense();
      auto      a  = agent.policy(current);
      try {
	simulator.timeStep(a);
	transition = TRANSITION(current,
				simulator.reward(),
				simulator.sense());
      }
      catch(rl::exception::Terminal& e) {
	transition = TRANSITION(current,simulator.reward());
      }
    }

    /**
     * This triggers an interaction from an agent and fill a
     * transition (next action in some non-terminal transition is
     * choosen by the agent itself). From this transition, agent
     * learning occurs.
     * @param continuation true means that s,a in the resulting transition is taken from the s'a' values of the transition given in argument. It allows to call the agent's policy once, only for a'.
     */
    template<typename SIMULATOR,typename AGENT,
	     typename ONLINE_CRITIC,typename TRANSITION>
    void adaptation(SIMULATOR& simulator,
		    const AGENT& agent,
		    ONLINE_CRITIC& critic,
		    TRANSITION& transition,
		    bool continuation) {
      typename TRANSITION::zset_type current,next;
	
      if(continuation) 
	current = transition.next();
      else 
	current = simulator.sense();
	
      try {
	simulator.timeStep(agent.policy(current));
	next = simulator.sense();
	transition = TRANSITION(current,
				simulator.reward(),
				next);
	critic.update(transition);
      }
      catch(rl::exception::Terminal& e) {
	transition = TRANSITION(current,simulator.reward());
	critic.update(transition);
      }
    }

    /**
     * This runs an episode.
     * @param max_episode_duration put a null or negative number to run the episode without length limitation.
     * @param length returns the actual episode length.
     */
    template<typename SIMULATOR,typename AGENT,typename TRANSITION>
    void run(SIMULATOR& simulator,
	     const AGENT& agent,
	     TRANSITION& transition,
	     int max_episode_duration,
	     int& length) {
      length=0;
      do {
	interaction(simulator,agent,transition);
	++length;
      } while(length != max_episode_duration && !transition.isTerminal());
    }

    /**
     * This appends the transitions of an episode in a set.
     * @param max_episode_duration put a null or negative number to run the episode without length limitation.
     * @param length returns the actual episode length.
     */
    template<typename SIMULATOR,typename AGENT,
	     typename ONLINE_CRITIC,typename TRANSITION>
    void run_and_learn(SIMULATOR& simulator,
		       const AGENT& agent,
		       ONLINE_CRITIC& critic,
		       TRANSITION& transition,
		       int max_episode_duration,
		       int& length) {
      auto current = simulator.sense();
      length=0;
      transition = TRANSITION(current, // dummy
			      0,       // dummy
			      current);
      do {
	adaptation(simulator,agent,critic,transition,true);
	++length;
      }
      while(length != max_episode_duration && !transition.isTerminal());
    }

    /**
     * This appends the transitions of an episode in a set.
     * @param max_episode_duration put a null or negative number to run the episode without length limitation.
     * @param length returns the actual episode length.
     */
    template<typename SIMULATOR,typename AGENT,typename TRANSITION_SET>
    void run_and_collect(SIMULATOR& simulator,
			 const AGENT& agent,
			 TRANSITION_SET& transition_set,
			 int max_episode_duration,
			 int& length) {
      typename TRANSITION_SET::value_type transition;
      length=0;
      do {
	interaction(simulator,agent,transition);
	transition_set.push_back(transition);
	++length;
      } while(length != max_episode_duration && !transition.isTerminal());
    }
      
    namespace sa {
      
      /**
       * This triggers an interaction from an action and fill a transition.
       */
      template<typename SIMULATOR,typename ACTION,typename SA_TRANSITION>
      void perform(SIMULATOR& simulator,
		   ACTION action,
		   SA_TRANSITION& transition) {
	auto current = simulator.sense();
	try {
	  simulator.timeStep(action);
	  transition = SA_TRANSITION(current,action,
				     simulator.reward(),
				     simulator.sense(),ACTION());
	}
	catch(rl::exception::Terminal& e) {
	  transition = SA_TRANSITION(current,action,simulator.reward());
	}
      }
      
      /**
       * This triggers an interaction from an agent and fill a transition.
       */
      template<typename SIMULATOR,typename AGENT,typename SA_TRANSITION>
      void interaction(SIMULATOR& simulator,
		       const AGENT& agent,
		       SA_TRANSITION& transition) {
	auto current = simulator.sense();
	auto       a = agent.policy(current);
	try {
	  simulator.timeStep(a);
	  transition = SA_TRANSITION(current,a,
				     simulator.reward(),
				     simulator.sense(),a /* dummy */);
	}
	catch(rl::exception::Terminal& e) {
	  transition = SA_TRANSITION(current,a,simulator.reward());
	}
      }
      
      /**
       * This triggers an interaction from an agent and fill a
       * transition (next action in some non-terminal transition is
       * choosen by the agent itself). From this transition, critic
       * learning occurs.
       * @param continuation true means that s,a in the resulting transition is taken from the s'a' values of the transition given in argument. It allows to call the agent's policy once, only for a'.
       */
      template<typename SIMULATOR,typename AGENT,
	       typename ONLINE_CRITIC, typename SA_TRANSITION>
      void adaptation(SIMULATOR& simulator,
		      const AGENT& agent,
		      ONLINE_CRITIC& critic,
		      SA_TRANSITION& transition,
		      bool continuation) {
	
	if(continuation) {
	  auto current = transition.nextState();
	  auto a       = transition.nextAction();

	  try {
	    simulator.timeStep(a);
	    auto next = simulator.sense();
	    transition = SA_TRANSITION(current,a,
				       simulator.reward(),
				       next,
				       agent.policy(next));
	    critic.update(transition);
	  }
	  catch(rl::exception::Terminal& e) {
	    transition = SA_TRANSITION(current,a,simulator.reward());
	    critic.update(transition);
	  }
	}
	else {
	  auto current = simulator.sense();
	  auto a       = agent.policy(current); 
	
	  // Duplicated code here.... due to the use of auto
	  try {
	    simulator.timeStep(a);
	    auto next = simulator.sense();
	    transition = SA_TRANSITION(current,a,
				       simulator.reward(),
				       next,
				       agent.policy(next));
	    critic.update(transition);
	  }
	  catch(rl::exception::Terminal& e) {
	    transition = SA_TRANSITION(current,a,simulator.reward());
	    critic.update(transition);
	  }
	}
      }
      
      /**
       * This appends the transitions of an episode in a set.
       * @param max_episode_duration put a null or negative number to run the episode without length limitation.
       * @param length returns the actual episode length.
       */
      template<typename SIMULATOR,typename AGENT,typename SA_TRANSITION>
      void run(SIMULATOR& simulator,
	       const AGENT& agent,
	       SA_TRANSITION& transition,
	       int max_episode_duration,
	       int& length) {
	length=0;
	do {
	  interaction(simulator,agent,transition);
	  ++length;
	} while(length != max_episode_duration && !transition.isTerminal());
      }
      
      /**
       * This appends the transitions of an episode in a set.
       * @param max_episode_duration put a null or negative number to run the episode without length limitation.
       * @param length returns the actual episode length.
       */
      template<typename SIMULATOR,typename AGENT,
	       typename ONLINE_CRITIC,typename SA_TRANSITION_SET>
      void run_and_learn(SIMULATOR& simulator,
			 const AGENT& agent,
			 ONLINE_CRITIC& critic,
			 SA_TRANSITION_SET& transition_set,
			 int max_episode_duration,
			 int& length) {
	typename SA_TRANSITION_SET::value_type transition;
	auto current = simulator.sense();
	auto       a = agent.policy(current);
	length=0;
	//transition = SA_TRANSITION_SET::value_type(current, // dummy
	//			   a,       // dummy
	//			   0,       // dummy
	//			   current,
	//			   a);
	do {
	  adaptation(simulator,agent,critic,transition,true);
	  transition_set.push_back(transition);
	  ++length;
	}
	while(length != max_episode_duration && !transition.isTerminal());
      }
      
      /**
       * This appends the transitions of an episode in a set.
       * @param max_episode_duration put a null or negative number to run the episode without length limitation.
       * @param length returns the actual episode length.
       */
      template<typename SIMULATOR,typename AGENT,typename SA_TRANSITION_SET>
      void run_and_collect(SIMULATOR& simulator,
			   const AGENT& agent,
			   SA_TRANSITION_SET& transition_set,
			   int max_episode_duration,
			   int& length) {
	typename SA_TRANSITION_SET::value_type transition;
	length=0;
	do {
	  interaction(simulator,agent,transition);
	  transition_set.push_back(transition);
	  ++length;
	} while(length != max_episode_duration && !transition.isTerminal());
      }
    }
  }
}

#endif
