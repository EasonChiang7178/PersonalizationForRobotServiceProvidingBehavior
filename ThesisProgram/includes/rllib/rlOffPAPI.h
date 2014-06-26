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

#ifndef rlOFFPAPI_H
#define rlOFFPAPI_H

namespace rl {
  namespace sa {
    
    /**
     * This performs a policy iteration.<br>
     */
    template<typename ARGMAX_BATCH_CRITIC, typename TRANSITION_ITERATOR>
    void batch_policy_iteration_step(ARGMAX_BATCH_CRITIC& critic,
				     const TRANSITION_ITERATOR& begin,
				     const TRANSITION_ITERATOR& end) {
      typename TRANSITION_ITERATOR::value_type::reward_type qmax;

      critic.update(begin,end);
      for(TRANSITION_ITERATOR iter = begin;iter != end;++iter)
	if(!((*iter).isTerminal()))
	  (*iter).setNextAction(critic.argmax((*iter).nextState(),qmax));
    }
  }
}

#endif
