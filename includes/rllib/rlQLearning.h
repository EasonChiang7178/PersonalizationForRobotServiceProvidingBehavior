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

#ifndef rlQLEARNING_H
#define rlQLEARNING_H

#include <rlException.h>
#include <sstream>

namespace rl {

  namespace exception {
    class QLearningBadParam : public Any {
    public:
      
      QLearningBadParam(std::string comment) 
	: Any(std::string("Bad theta parameter in QLearning: ")+comment) {}
    };

  }

  namespace sa {
    /**
     * @short QLearning algorithm
     */
    template<typename SA_ARCHITECTURE,
	     typename SA_GREEDY,
	     typename SARSA_PARAM>
    class QLearning 
      : public rl::Serializer {
    public: 


    private:
    
      const SA_ARCHITECTURE& archi;
      const SA_GREEDY&       greedy;
      const SARSA_PARAM&     param;
      gsl_vector* theta;
      gsl_vector* grad;

    void paramCopy(const gsl_vector* th, const gsl_vector* gr) {
      gsl_vector_free(theta);
      gsl_vector_free(grad);
      theta = 0;
      grad = 0;
      
      if(th != 0) {
	theta = gsl_vector_alloc(th->size);
	gsl_vector_memcpy(theta, th);
      }
      
      if(gr != 0) {
	grad = gsl_vector_alloc(gr->size);
	gsl_vector_memcpy(grad, gr);
      }
    }

    protected:


      virtual void read(std::istream& is) {
	is >> theta;
      }
      
      virtual void write(std::ostream& os) const {
	os << theta;
      }

    public:

      typedef QLearning<SA_ARCHITECTURE,SA_GREEDY,SARSA_PARAM> self_type;
      typedef typename SA_ARCHITECTURE::input_type  input_type;
      typedef typename SA_ARCHITECTURE::output_type output_type;
      typedef typename SA_ARCHITECTURE::state_type  state_type;
      typedef typename SA_ARCHITECTURE::action_type action_type;

      /**
       * Get the architecture parameter computed by QLearning.
       */
      const gsl_vector* parameter(void) const {
	return theta;
      }

      /**
       * This sets (by a copy) the vector parameter.
       */
      void operator=(const gsl_vector* new_theta) {
	if(new_theta == (gsl_vector*)0) 
	  throw rl::exception::QLearningBadParam("theta = NULL");
	else if(new_theta->size != theta->size) {
	  std::ostringstream ostr;
	  ostr << "argument size is " << new_theta->size 
	       << " but sould be " << theta->size << ".";
	  throw rl::exception::QLearningBadParam(ostr.str());
	}

	gsl_vector_memcpy(theta,new_theta);
      }

      QLearning(const SA_ARCHITECTURE& a,
		const SA_GREEDY&       g,
		const SARSA_PARAM&     p)
	: archi(a), 
	  greedy(g),
	  param(p),
	  theta(archi.newParameterInstance()),
	  grad(archi.newParameterInstance()) {
      }

      QLearning(const self_type& cp)
	: rl::Serializer(cp),
	  archi(cp.archi), 
	  greedy(cp.greedy),
	  param(cp.param),
	  theta(0),
	  grad(0) {
	paramCopy(cp.theta, cp.grad);
      }


      self_type& operator=(const self_type& cp) {
	if(this != &cp) {
	  archi = cp.archi; 
	  greedy = cp.greedy;
	  param = cp.param;
	  paramCopy(cp.theta, cp.grad);
	}
	return *this;
      }

      ~QLearning(void) {
	gsl_vector_free(theta);
	gsl_vector_free(grad);
      }
                   
      output_type operator()(const input_type &sa) const {
	return archi(theta,sa);
      }

      output_type operator()(const state_type &s, const action_type &a) const {
	return archi(theta,s,a);
      }

      template<typename TRANSITION>
      void update(const TRANSITION &transition){
	output_type qsa,next_qsa,td;

	qsa      = archi(theta,
			 transition.currentState(),
			 transition.currentAction());
	archi.gradient(theta,
		       transition.currentState(),
		       transition.currentAction(),
		       grad);

	if(transition.isTerminal())
	  td = transition.reward() - qsa;
	else {
	  greedy(archi,theta,transition.nextState(),next_qsa);
	  td = transition.reward() + param.gamma()*next_qsa - qsa;
	}
      
	// theta <- theta + alpha*td*grad
	gsl_blas_daxpy(td*param.alpha(),
		       grad,
		       theta);
      }

      template<typename TRANSITION_ITERATOR>
      void update(const TRANSITION_ITERATOR& begin, const TRANSITION_ITERATOR& end) {
	for(TRANSITION_ITERATOR i=begin; i!=end; ++i)
	  update(*i);
      }

      action_type argmax(const state_type &s, output_type &max) const {
	return greedy(archi,theta,s,max);
      }
    };
    
    template<typename SA_ARCHITECTURE,
	     typename SA_GREEDY,
	     typename SARSA_PARAM>
    QLearning<SA_ARCHITECTURE,SA_GREEDY,SARSA_PARAM> q_learning(const SA_ARCHITECTURE& a,
								const SA_GREEDY&       g,
								const SARSA_PARAM&     p) {
      return QLearning<SA_ARCHITECTURE,SA_GREEDY,SARSA_PARAM>(a,g,p);
    }
    
  }
}


#endif
