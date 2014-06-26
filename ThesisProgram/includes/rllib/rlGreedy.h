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

#ifndef rlGREEDY_H
#define rlGREEDY_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <rlConcept.h>
#include <rlRandom.h>

namespace rl {
  namespace sa {

    /**
     * @short A greedy functor for iterable actions.
     *
     * ITERATOR fits rl::concept::Iterator<SA_ARCHITECTURE::action_type>
     */
    template<typename ITERATOR>
    class GreedyFromAIteration {
    private:
      const ITERATOR& action_iter;

    public:
      
      GreedyFromAIteration(const ITERATOR& iter) : action_iter(iter) {}

      template<typename SA_ARCHITECTURE>
      typename SA_ARCHITECTURE::action_type operator()(const SA_ARCHITECTURE& archi,
						       const typename SA_ARCHITECTURE::param_type& theta,
						       const typename SA_ARCHITECTURE::state_type& s,
						       typename SA_ARCHITECTURE::output_type& max) const {
	typename SA_ARCHITECTURE::action_type a_max;
	typename SA_ARCHITECTURE::output_type q;
	typename SA_ARCHITECTURE::action_type a(action_iter.first());

	a_max = a;
	max = archi(theta,s,a);
	while(action_iter.hasNext(a)) {
	  a = action_iter.next(a);
	  q = archi(theta,s,a);
	  if(q>max) {
	    max   = q;
	    a_max = a;
	  }
	}
	return a_max;
      }
    };
    
    template<typename ITERATOR>
    GreedyFromAIteration<ITERATOR> greedy(const ITERATOR& iter) {
      return GreedyFromAIteration<ITERATOR>(iter);
    }

    /**
     * @short This adds the rl::concept::sa::ArgmaxCritic interface to a function.
     *
     * ITERATOR fits rl::concept::Iterator<SA_FUNCTION::action_type>
     */
    template<typename SA_FUNCTION, typename ITERATOR>
    class ArgmaxFromAIteration : public SA_FUNCTION {
    private:
      ITERATOR action_iter;

    public:
      typedef typename SA_FUNCTION::state_type  state_type;
      typedef typename SA_FUNCTION::action_type action_type;
      typedef typename SA_FUNCTION::output_type output_type;
      
      ArgmaxFromAIteration(const SA_FUNCTION& f, const ITERATOR& iter) : SA_FUNCTION(f), action_iter(iter) {}

      action_type argmax(const state_type& s, output_type& max) const {
	action_type a_max;
	output_type q;
	action_type a(action_iter.first());

	a_max = a;
	max = (*this)(s,a);
	while(action_iter.hasNext(a)) {
	  a = action_iter.next(a);
	  q = (*this)(s,a);
	  if(q>max) {
	    max   = q;
	    a_max = a;
	  }
	}
	return a_max;
      }
    };

    template<typename SA_FUNCTION, typename ITERATOR>
    ArgmaxFromAIteration<SA_FUNCTION,ITERATOR> argmax(const SA_FUNCTION& f, const ITERATOR& iter) {
      return ArgmaxFromAIteration<SA_FUNCTION,ITERATOR>(f,iter);
    }
  }
}

#endif
