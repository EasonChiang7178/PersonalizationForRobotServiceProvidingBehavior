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

#ifndef rlAGENT_H
#define rlAGENT_H

#include <rlRandom.h>
#include <rlTypes.h>
#include <rlConcept.h>
#include <vector>
#include "../../Human-AwareInteractiveLearner/rl-InteractiveLearner.h"

namespace rl {

  namespace agent {

    /**
     * @short This builds a greedy agent from an existing critic instance.
     */
    template<typename SA_ARGMAX_FUNCTION>
    class Greedy {
    private:
      
      const SA_ARGMAX_FUNCTION& that;

      Greedy(void) {};
      Greedy<SA_ARGMAX_FUNCTION>& operator=(const Greedy<SA_ARGMAX_FUNCTION>&) {};

    public:
      typedef typename SA_ARGMAX_FUNCTION::state_type  state_type;
      typedef typename SA_ARGMAX_FUNCTION::action_type action_type;
      typedef typename SA_ARGMAX_FUNCTION::output_type output_type;
	
      Greedy(const SA_ARGMAX_FUNCTION& critic) : that(critic) {}
      Greedy(const Greedy<SA_ARGMAX_FUNCTION>& cp) : that(cp.that) {}

      action_type policy(const state_type& s) const {
	output_type dummy;
	return that.argmax(s,dummy);
      }
    };

    template<typename SA_ARGMAX_FUNCTION>
    Greedy<SA_ARGMAX_FUNCTION> greedy(const SA_ARGMAX_FUNCTION& critic) {
      return Greedy<SA_ARGMAX_FUNCTION>(critic);
    }

    
    /**
     * @short This builds an epsilon-greedy agent from an existing critic instance.
     */
    template<typename SA_ARGMAX_FUNCTION,
	     typename TOSS,
	     typename EPSILON_GREEDY_PARAM>
    class EpsilonGreedy {
    private:
      
      const SA_ARGMAX_FUNCTION& that;
      const TOSS& tosser;
      const EPSILON_GREEDY_PARAM& param;
	
      EpsilonGreedy(void) {}; 
      EpsilonGreedy<SA_ARGMAX_FUNCTION,TOSS,EPSILON_GREEDY_PARAM>& operator=(const EpsilonGreedy<SA_ARGMAX_FUNCTION,TOSS,EPSILON_GREEDY_PARAM>&) {}

    public:
      typedef typename SA_ARGMAX_FUNCTION::state_type  state_type;
      typedef typename SA_ARGMAX_FUNCTION::action_type action_type;
      typedef typename SA_ARGMAX_FUNCTION::output_type output_type;
	
      EpsilonGreedy(const SA_ARGMAX_FUNCTION& critic,
		    const TOSS& toss,
		    const EPSILON_GREEDY_PARAM& p) 
	: that(critic), tosser(toss), param(p) {}
      EpsilonGreedy(const EpsilonGreedy<SA_ARGMAX_FUNCTION,TOSS,EPSILON_GREEDY_PARAM>& cp) 
	: that(cp.that), tosser(cp.tosser), param(cp.param) {}

      action_type policy(const state_type& s) const {
	output_type dummy;
	  
	if(rl::random::toss() < param.epsilon())
	  return tosser.random();
	else
	  return that.argmax(s,dummy);
      }
    };

    template<typename SA_ARGMAX_FUNCTION,
	     typename TOSS,
	     typename EPSILON_GREEDY_PARAM>
    EpsilonGreedy<SA_ARGMAX_FUNCTION,TOSS,EPSILON_GREEDY_PARAM> epsilon_greedy(const SA_ARGMAX_FUNCTION& critic,
									       const TOSS& toss,
									       const EPSILON_GREEDY_PARAM& p) {
      return EpsilonGreedy<SA_ARGMAX_FUNCTION,TOSS,EPSILON_GREEDY_PARAM>(critic,toss,p);
    }
    

      
    /**
     * @short This builds an agent that behaves randomly.
     *
     * TOSS is the action tosser.
     */
    template<typename STATE,typename TOSS>
    class Random {
    private:
      
      const TOSS& tosser;
      
    public:

      typedef typename TOSS::value_type action_type;
      typedef STATE                     state_type;
    
      Random(const TOSS& toss) : tosser(toss) {}
    
      action_type policy(const state_type& s) const {
	return tosser.random();
      }
    };
  
    /**
     * @param s Provide here any instance of state.
     */
    template<typename STATE,typename TOSS>
    Random<STATE,TOSS> random(const STATE& s,const TOSS& toss) {
      return Random<STATE,TOSS>(toss);
    }
  
    /**
     * @short This builds a soft-max agent from an existing critic instance.
     */
    template<typename SA_FUNCTION, 
	     typename ACTION_SIZED_ITERATOR,
	     typename SOFTMAX_PARAM>
    class SoftMax {
    private:
      
      const SA_FUNCTION& that;
      const ACTION_SIZED_ITERATOR& iter;
      const SOFTMAX_PARAM& param;
      std::vector<double> cumul;
      SoftMax(void) {};
      SoftMax<SA_FUNCTION,ACTION_SIZED_ITERATOR,SOFTMAX_PARAM>& operator=(const SoftMax<SA_FUNCTION,ACTION_SIZED_ITERATOR,SOFTMAX_PARAM>&) {};

    public:
      typedef typename SA_FUNCTION::state_type  state_type;
      typedef typename SA_FUNCTION::action_type action_type;
      typedef typename SA_FUNCTION::output_type output_type;
	
      SoftMax(const SA_FUNCTION& critic,
	      const ACTION_SIZED_ITERATOR& aiter,
	      const SOFTMAX_PARAM& p) : that(critic), iter(aiter), param(p), cumul() {}
      SoftMax(const SoftMax<SA_FUNCTION,ACTION_SIZED_ITERATOR,SOFTMAX_PARAM>& cp) 
	: that(cp.that), iter(cp.iter), param(cp.param), cumul(cp.cumul) {}

      action_type policy(const state_type& s) const {
	std::vector<double>& cum = const_cast<std::vector<double>&>(cumul);
	cum.resize(iter.size());
	action_type a = iter.first();
	int i=0;
	double value;

	cum[i] = exp(that(s,a)/param.temperature());
	while(iter.hasNext(a)) {
	  ++i;
	  a = iter.next(a);
	  cum[i] = exp(that(s,a)/param.temperature()) + cum[i-1];
	}
	value = rl::random::toss(0,cum[iter.size()-1]);
	for(i=0;value>=cum[i];++i);
	return iter.value(i);
      }
    };

    template<typename SA_FUNCTION, 
	     typename ACTION_SIZED_ITERATOR,
	     typename SOFTMAX_PARAM>
    SoftMax<SA_FUNCTION,ACTION_SIZED_ITERATOR,SOFTMAX_PARAM> softmax(const SA_FUNCTION& critic,
								      const ACTION_SIZED_ITERATOR& aiter,
								      const SOFTMAX_PARAM& p) {
      return SoftMax<SA_FUNCTION,ACTION_SIZED_ITERATOR,SOFTMAX_PARAM>(critic,aiter,p);
    }

		/**
		 * @short This builds a Diberate agent from an existing critic instance (Diberate Policy of Action Selection)
		 */
		template<typename SA_FUNCTION, 
				 typename ACTION_SIZED_ITERATOR,
				 typename DPAS_PARAM>
		class DiberateActionSelection {
			private:
				const SA_FUNCTION& that;
				const ACTION_SIZED_ITERATOR& iter;
				const DPAS_PARAM& param;
				std::vector<double> cumul;
				DiberateActionSelection(void) {};
				DiberateActionSelection<SA_FUNCTION,ACTION_SIZED_ITERATOR,DPAS_PARAM>& operator=(const DiberateActionSelection<SA_FUNCTION,ACTION_SIZED_ITERATOR,DPAS_PARAM>&) {};

			public:
				typedef typename SA_FUNCTION::state_type  state_type;
				typedef typename SA_FUNCTION::action_type action_type;
				typedef typename SA_FUNCTION::output_type output_type;
		
				DiberateActionSelection(const SA_FUNCTION& critic,
										const ACTION_SIZED_ITERATOR& aiter,
										const DPAS_PARAM& p) : that(critic), iter(aiter), param(p), cumul() {}
				DiberateActionSelection(const SoftMax<SA_FUNCTION,ACTION_SIZED_ITERATOR,DPAS_PARAM>& cp)
									    : that(cp.that), iter(cp.iter), param(cp.param), cumul(cp.cumul) {}

				action_type policy(const state_type& s) const {
					std::vector<double>& cum = const_cast<std::vector<double>&>(cumul);
					action_type a = iter.first();
					if (s.robotBelief != AdaptiveInterruption::DoesPersonIgnoreMe && s.robotBelief != AdaptiveInterruption::DoesPersonLookingMe) {
						cum.resize(8);
						int i = 0;
						double value;

						cum[i] = exp(that(s,a)/param.temperature());
						while (iter.next(a) != rl::problem::adaptive_interruption::StraightStyle) {
							++i;
							a = iter.next(a);
							cum[i] = exp(that(s,a)/param.temperature()) + cum[i-1];
						}
						value = rl::random::toss(0,cum[7]);
						for (i = 0; value >= cum[i]; ++i);
							return iter.value(i);
					}
					else {
						cum.resize(3);
						for (int i = 0; i < 11; i++)
							a = iter.next(a);

						int i = 0;
						double value;

						cum[i] = exp(that(s,a)/param.temperature());
						while (iter.hasNext(a)) {
							++i;
							a = iter.next(a);
							cum[i] = exp(that(s,a)/param.temperature()) + cum[i-1];
						}
						value = rl::random::toss(0,cum[2]);
						for (i = 0; value >= cum[i]; ++i);
							return iter.value(i + 8);
					}
				}
		};

		template<typename SA_FUNCTION, 
				 typename ACTION_SIZED_ITERATOR,
				 typename DPAS_PARAM>
		DiberateActionSelection<SA_FUNCTION,ACTION_SIZED_ITERATOR,DPAS_PARAM> diberateactionselection(const SA_FUNCTION& critic,
																									  const ACTION_SIZED_ITERATOR& aiter,
																									  const DPAS_PARAM& p) {
			return DiberateActionSelection<SA_FUNCTION,ACTION_SIZED_ITERATOR,DPAS_PARAM>(critic,aiter,p);
		}
	}
}

#endif
