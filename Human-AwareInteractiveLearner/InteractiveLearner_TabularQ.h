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


typedef rl::Iterator<A,
			 rl::problem::adaptive_interruption::Approach,
			 rl::problem::adaptive_interruption::Joke> ActionIterator;
typedef ActionIterator													ActionToss;


// Let us define an architecture that represents the tabular coding of. 
// It fits rl::concept::sa::DerivableArchitecture. In usual rl algorithms,
// parameters are represented as a gsl_vector.
class TabularQ {
	public:
		typedef S			state_type;
		typedef A			action_type;
		typedef gsl_vector*	param_type;
		typedef SA 			sa_type;
		typedef SA 			input_type;
		typedef Reward		output_type;

		param_type newParameterInstance(void) const {
				// vector size is |S|*|A|
			return gsl_vector_calloc(AdaptiveInterruption::size * rl::problem::adaptive_interruption::actionSize);
		}

		output_type operator()(const param_type theta, const input_type &sa) const {
			return (*this)(theta,sa.s,sa.a);
		}

		output_type operator()(const param_type theta, state_type s, action_type      a) const {
			// This corresponds to the reading of the appropriate component of theta.
			return gsl_vector_get(theta, (a * AdaptiveInterruption::ALRSize * AdaptiveInterruption::ToASize) + 
										 (s.robotBelief * AdaptiveInterruption::ALRSize) +
										 (s.personAttention));
		}

		void gradient(const param_type  theta, const input_type& sa, param_type        grad) const {
			gradient(theta,sa.s,sa.a,grad);
		}

		void gradient(const param_type theta, state_type       s, action_type      a, param_type       grad) const {
				// This is a 00001000....00 vector with a single value 1 at the
				// index of the active sa pair.
			gsl_vector_set_basis(grad, (a * AdaptiveInterruption::ALRSize * AdaptiveInterruption::ToASize) + 
									   (s.robotBelief * AdaptiveInterruption::ALRSize) +
									   (s.personAttention));
		}
};
