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

#ifndef rlCONCEPT_H
#define rlCONCEPT_H

#include <gsl/gsl_vector.h>
#include <vector>

namespace rl {
  
  /**
   * @short Helps doxygen to document that a concept extends another one.
   */
  template<typename ANY>
  class extends {
  };

  template<typename STATE, typename ACTION> class SA;
  template<typename INPUT,typename REWARD>  class Transition;

  namespace concept {


    /**
     * @short <b>This should behave like a scalar (+, <, ...) </b>
     *
     * This is used for reward and returns.
     */
    class Reward {
    public:
      
    };

    /**
     * @short State space
     *
     * This is the state space used in the MDP models.
     */
    class State {
    public:
      State(void);
      State(const State& s);
      State& operator=(const State& s);
    };

    /**
     * @short Observation space
     *
     * This is the observation that is returned from sensors of the controlled system.
     */
    class Observation {
    public:
      Observation(void);
      Observation(const Observation& s);
      Observation& operator=(const Observation& s);
    };

    /**
     * @short Phase space
     *
     * This is the phase space of the controlled system.
     */
    class Phase {
    public:
      Phase(void);
      Phase(const Phase& s);
      Phase& operator=(const Phase& s);
    };

    /**
     * @short This is what is evaluated (S, SxA, ...).
     * 
     * When something, applies both to s and (s,a), s or (s,a) is
     * sometimes noted z.
     */
    class Zset {
    public:
      Zset(void);
      Zset(const Zset& z);
      Zset& operator=(const Zset& z);
    };

    /**
     * @short This is an online algorithm.
     */
    template<typename TRANSITION>
    class Online {
    public:
      /**
       * This call modifies some internal data according to the
       * provided transition. This modification is what the algorithm
       * actually computes. An online algorithm performs successive
       * updated for a serie of transitions it receives.
       */
      void update(const TRANSITION& transition);
    };

    /**
     * @short This is a batch algorithm.
     */
    template<typename TRANSITION_ITERATOR>
    class Batch {
    public:
      /**
       * This call modifies some internal data according to the
       * provided transition set. This modification is what the
       * algorithm actually computes.
       */
      void update(const TRANSITION_ITERATOR& begin,
		  const TRANSITION_ITERATOR& end);
    };

    /**
     * @short This is the agent.
     */
    template<typename STATE,typename ACTION>
    class Agent {
    public:
      typedef STATE  state_type;
      typedef ACTION action_type;
      
      /**
       * This function tells what action has to be performed when the
       * agent is at state s.
       */
      action_type policy(const state_type& s) const;
    };

    /**
     * @short Action space
     *
     * This is the action space in which the command is expressed.
     */
    class Action {
    public:
      Action(void);
      Action(const Action& a);
      Action& operator=(const Action& a);
    };

    class Input {
    public:
    };

    class Output {
    public:
    };


    /**
     * @short Upper concept for a relation between an input set (zset) and an output set (co-zset).
     *
     * In reinforcement, zset and co-zset are induced by the kind of transition.
     */
    template<typename INPUT,typename OUTPUT>
    class Relation {
    public:
      typedef INPUT  input_type;
      typedef OUTPUT output_type;
    };

    namespace sa {
      /**
       * @short Specification of rl::concept::Relation for state/action transitions.
       */
      template<typename STATE, typename ACTION, typename OUTPUT>
      class Relation : public concept::Relation<rl::SA<STATE,ACTION>,OUTPUT> {
      public:
	typedef STATE  state_type;
	typedef ACTION action_type;
      };
    }
    

    /**
     * @short Concept for functions.
     */
    template<typename INPUT,typename OUTPUT>
    class Function : public Relation<INPUT,OUTPUT> {
    public:
      typedef INPUT  input_type;
      typedef OUTPUT output_type;

      /**
       * Function call 
       */
      output_type operator()(const input_type& x) const;
    };

    namespace sa {
      /**
       * @short Concept for functions using state/action pair.
       */
      template<typename STATE, typename ACTION, typename OUTPUT>
      class Function 
	: public sa::Relation<STATE,ACTION,OUTPUT>,
	  public concept::Function<rl::SA<STATE,ACTION>,OUTPUT> {
      public:
	typedef STATE  state_type;
	typedef ACTION action_type;
	typedef OUTPUT output_type;
	
	/**
	 * Function call 
	 */
	output_type operator()(const state_type& s, const action_type& a) const;
      };
    }


    /**
     * @short This is for feature space mapping.
     *
     * A feature space is a space where each input is represented as a finite dimension vector. 
     */
    template<typename INPUT>
    class Feature {
    public:
      typedef INPUT input_type;

      /**
       * This return the dimension of the feature space.
       */
      int dimension(void) const;

      /**
       * This computes the vector associated to some input.
       * @param x is the input/
       * @param phi is the 'dimension'-sized vector (that is set by the call) corresponding to x.
       */
      void operator()(const input_type& x,
		      gsl_vector* phi) const;
    };

    namespace sa {
      /**
       * @short This is for feature space mapping for state/action pairs.
       */
      template<typename STATE, typename ACTION>
      class Feature : public concept::Feature< rl::SA<STATE,ACTION> > {
      public:
	typedef STATE  state_type;
	typedef ACTION action_type;
	
      /**
       * This computes the vector associated to some input.
       * @param s is the input state.
       * @param a is the input action.
       * @param phi is the 'dimension'-sized vector (that is set by the call) corresponding to (s,a).
       */
	void operator()(const state_type& s,
			const action_type& a,
			gsl_vector* phi) const;
	
      };

    }

    /**
     * @short Concept for parametrized function architectures.
     *
     * An architecture is an object that behaves as a function, once
     * it is provided with some parameter.
     */ 
    template<typename INPUT,typename OUTPUT, typename ANY>
    class Architecture : public Relation<INPUT,OUTPUT> {
    public:
      typedef INPUT  input_type;
      typedef OUTPUT output_type;
      typedef ANY    param_type;

      Architecture(void);

      /**
       * @param theta is the parameter.
       * @param x is the function argument.
       * @return the value returned by the function when applied to x, knowing theta.
       */
      output_type operator()(const param_type& theta,
			     const input_type& x) const;

      /**
       * In some algorithms, it is needed to allocate a new parameter
       * value by askin an instance to some architecture object. This
       * is what this method does.
       */
      param_type newParameterInstance(void) const;
    };

    namespace sa {
      /**
       * @short Concept for parametrized function architectures with state/action inut.
       */ 
      template<typename STATE, typename ACTION, typename OUTPUT, typename ANY>
      class Architecture 
	: public sa::Relation<STATE,ACTION,OUTPUT>,
	  public concept::Architecture<rl::SA<STATE,ACTION>,OUTPUT,ANY> {
      public:
	typedef STATE  state_type;
	typedef ACTION action_type;
	typedef OUTPUT output_type;
	typedef ANY    param_type;

	/**
	 * @param theta is the parameter.
	 * @param s is the state argument.
	 * @param a is the action argument.
	 * @return the value returned by the function when applied to (s,a), knowing theta.
	 */
	output_type operator()(const param_type& theta,
			       const state_type& s, 
			       const action_type& a) const;
      };
    }

    /**
     * @short Concept for architectures for which gradient according to parameters is known.
     */ 
    template<typename INPUT,typename OUTPUT, typename ANY>
    class DerivableArchitecture : public Architecture<INPUT,OUTPUT,ANY> {
    public:
      typedef INPUT input_type;
      typedef ANY   param_type;

      /**
       * @param theta is the parameter.
       * @param x is the value at which the gradient is considered.
       * @param grad this parameter is set as the gradient value at x.
       */
      void gradient(const param_type& theta,
		    const input_type& x,
		    param_type& grad) const;

    };

    namespace sa {
      /**
       * @short Concept for state/action architectures for which gradient according to parameters is known.
       */ 
      template<typename STATE, typename ACTION, typename OUTPUT, typename ANY>
      class DerivableArchitecture 
	: public sa::Architecture<STATE,ACTION,OUTPUT,ANY>,
	  public concept::DerivableArchitecture<rl::SA<STATE,ACTION>,OUTPUT,ANY> {
      public:
	typedef STATE  state_type;
	typedef ACTION action_type;
	typedef ANY    param_type;

	/**
	 * @param theta is the parameter.
	 * @param s is the state value at which the gradient is considered.
	 * @param a is the action value at which the gradient is considered.
	 * @param grad this parameter is set as the gradient value at (s,a).
	 */
	void gradient(const param_type& theta,
		      const state_type& s, 
		      const action_type& a,
		      param_type& grad) const;
      };
    }

    template<typename INPUT,typename OUTPUT, typename ANY>
    class Critic 
      : public Architecture<INPUT,OUTPUT,ANY> {
    };

    /**
     * @short This implements some critic that learns online.
     */
    template<typename TRANSITION, typename INPUT,typename OUTPUT,typename ANY>
    class OnlineCritic
      : public Critic<INPUT,OUTPUT,ANY>, 
	public Online<TRANSITION> {
    public:
    };

    /**
     * @short This implements some critic that learns offline.
     */
    template<typename INPUT,typename OUTPUT,typename ANY, 
	     typename TRANSITION_ITERATOR>
    class BatchCritic
      : public Critic<INPUT,OUTPUT,ANY>, 
	public Batch<TRANSITION_ITERATOR>  {
    public:
    };
    
    namespace sa {

      /**
       * @short This define something for which an argmax action is meaningful.
       */
      template<typename STATE, typename ACTION, typename OUTPUT>
      class Argmax : public sa::Relation<STATE,ACTION,OUTPUT> {
      public:
	typedef STATE  state_type;
	typedef ACTION action_type;
	typedef OUTPUT output_type;
	
	/**
	 * @param s this is the state from which the best action has to be found.
	 * @param max this is the value of the best action from s.
	 * @return the best action from s.
	 */ 
	action_type argmax(const state_type& s, output_type& max) const;
      };

      
      /**
       * @short This implements a function, for which a argmax_a Q(s,a) is available
       */
      template<typename STATE, typename ACTION, typename OUTPUT>
      class ArgmaxFunction 
	: public sa::Function<STATE,ACTION,OUTPUT>,
	  public Argmax<STATE,ACTION,OUTPUT> {
      };
      
      

      template<typename STATE, typename ACTION, typename OUTPUT, typename ANY>
      class Critic 
	: public sa::Architecture<STATE,ACTION,OUTPUT,ANY> {
      };
      
      /**
       * @short This implements some critic that learns online.
       */
      template<typename TRANSITION, typename STATE, typename ACTION, typename OUTPUT,typename ANY>
      class OnlineCritic
	: public sa::Critic<STATE,ACTION,OUTPUT,ANY>,
	  public concept::OnlineCritic<TRANSITION,rl::SA<STATE,ACTION>,OUTPUT,ANY> {
      public:
      };
      
      /**
       * @short This implements some critic that learns offline.
       */
      template<typename TRANSITION, typename STATE, typename ACTION, typename OUTPUT,typename ANY>
      class BatchCritic
	: public sa::Critic<STATE,ACTION,OUTPUT,ANY>,
	  public concept::BatchCritic<TRANSITION,rl::SA<STATE,ACTION>,OUTPUT,ANY> {
      public:
      };

      
      /**
       * @short This implements a critic, for which a argmax_a Q(s,a) is available
       */
      template<typename STATE, typename ACTION, typename OUTPUT, typename ANY>
      class ArgmaxCritic 
	: public sa::Critic<STATE,ACTION,OUTPUT,ANY>,
	  public sa::Argmax<STATE,ACTION,OUTPUT> {
      };

      /**
       * @short This implements process that finds the best action.
       * The best action is determine according to an architecture and a parameter.
       */
      template<typename SA_ARCHITECTURE>
      class Greedy {
      public:
	
	Greedy(void);

	/**
	 * @param archi the architecture providing a value archi(theta,s,a) for each state/action pair.
	 * @param theta the considered parameter
	 * @param s the considered state
	 * @param max the value of the best action, as evaluated by the architecture and theta.
	 * @return the best action.
	 */
	typename SA_ARCHITECTURE::action_type operator()(const SA_ARCHITECTURE& archi,
							 const typename SA_ARCHITECTURE::param_type& theta,
							 const typename SA_ARCHITECTURE::state_type& s,
							 typename SA_ARCHITECTURE::output_type& max) const;
      };

    }

    namespace param {
      /**
       * @short The gamma parameter.
       */
      class Gamma {
      public:
	/**
	 * The discount factor value
	 */
	double gamma(void) const;
      };

      /**
       * @short The epsilon-greedy parameter.
       */
      class EpsilonGreedy {
      public:
	/**
	 * The epsilon value for epsilon-greedy operations.
	 */
	double epsilon(void) const;
      };

      /**
       * @short TD learning parameters.
       */
      class TD : public Gamma {
      public:
	/**
	 * The learning rate
	 */
	double alpha(void) const;
      };

      /**
       * @short LSTD parameters.
       */
      class LSTD : public Gamma {
      public:
	/**
	 * The regulation factor for batch lstd;
	 */
	double reg(void) const;
      };

      /**
       * @short KTD parameters.
       */
      class KTD : public Gamma {
      public:
	double eta_noise(void) const;
	double observation_noise(void) const;
	double priorVar(void) const;
	double h(void) const;
	double randomAmplitude(void) const;
      };

      /**
       * @short This is a saturated transfer function gain parameter.
       */
      class Saturation {
      public:
	double a(void) const;
      };

      /**
       * @short This is the soft-max temperature param.  
       **/
      class SoftMax {
      public:
	double temperature(void) const;
      };
    }

    /**
     * @short The simulator itself
     *
     * A simulator is the external world, or the dynamical process
     * that is controlled by the agent through successive actions. It
     * also includes the reward. Its current state is calle the
     * current phase, to avoid ambiguity with the state in
     * reinforcement learning. Indeed, the state space is a model use
     * by the agent to represent the phase of the simulator. From each
     * phase, an observation can be provided, as with sensors for a
     * real robotic system. In most simulated cases, phase and state
     * are the same, as well as observation if observable markovian
     * processes are used.
     */
    template <typename PHASE,typename ACTION,typename OBSERVATION>
    class Simulator {
    public:
      
      typedef PHASE       phase_type;
      typedef ACTION      action_type;
      typedef OBSERVATION observation_type;
      typedef double      reward_type;
      

    public:

      Simulator(void);
      ~Simulator(void);
      
      /**
       * This gives the observation corresponding to current phase.
       */
      const observation_type& sense(void) const;

      /**
       * This triggers a transition to a new phase, consecutivly to action a. This call may raise a rl::exception::Terminal if some terminal state is reached.
       */
      void timeStep(const action_type& a);

      /**
       * This gives the reward obtained from the last phase transition.
       */
      reward_type reward(void) const;
    };

    /**
     * This allows to iterate on the values from an enumerated type.
     */
    template<typename ANY>
    class Iterator {
    public:

      typedef ANY value_type;

      Iterator(const Iterator<ANY>& cp);

      /**
       * @return the first value of the enumerated type.
       */
      ANY  first(void) const;

      /**
       * Tells whether there is another value in the type, after
       * current. This is used with next method to iterate on all the
       * values in the type enumeration.
       */
      bool hasNext(const ANY& current) const;

      /**
       * @returns the value after current in the type
       * enumeration. This is used with hasNext method to iterate on
       * all the values in the type enumeration.
       */ 
      ANY  next(const ANY& current) const;
    };

    /**
     * This behaves as a vector of values for iteration.
     */
    template<typename ANY>
    class SizedIterator : public Iterator<ANY> {
    public:
      
      /**
       * Returns the number of values.
       */
      int size(void) const;

      /**
       * @param rank in [0,size()[
       * @return the (rank+1)-th value of the enumerated type.
       */
      ANY  value(int rank) const;
    };

    /**
     * This allows to get a random value
     */
    template<typename ANY>
    class Toss {
    public:
      typedef ANY any_type;

      /**
       * @returns a random value belonging to the set any_type.
       */
      ANY random(void) const;
    };


    namespace mlp {
      /**
       * @short This is a transfer function for a neuron.
       */
      class Transfer {
      public:
	/**
	 * @params  weighted_sum is sum wi.xi for a neuron
	 * @returns y = f(sum wi.xi).
	 */
	double operator()(double weighted_sum) const;
	std::string name(void) const;
      };
    }

  }

}


#endif
