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

#ifndef rlSARSA_H
#define rlSARSA_H

#include <rlException.h>
#include <sstream>

namespace rl {

  namespace exception {
    class SARSABadParam : public Any {
    public:
      
      SARSABadParam(std::string comment) 
	: Any(std::string("Bad theta parameter in SARSA: ")+comment) {}
    };

  }

  /**
   * @short TD algorithm
   *
   */
  
  template<typename ARCHITECTURE,
	   typename TD_PARAM>
  class TD 
    : public rl::Serializer { 
  public:
  
    typedef TD<ARCHITECTURE,TD_PARAM> self_type;

      typedef typename ARCHITECTURE::input_type  input_type;
      typedef typename ARCHITECTURE::output_type output_type;

  protected:
    
    const ARCHITECTURE& archi;
    TD_PARAM     td_param;
    gsl_vector* theta;
    gsl_vector* grad;

    virtual void read(std::istream& is) {
      is >> theta;
    }
    
    virtual void write(std::ostream& os) const {
      os << theta;
    }

  private:

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
    
    
  public:


    /**
     * Get the architecture parameter computed by TD.
     */
    const gsl_vector* parameter(void) const {
      return theta;
    }

    /**
     * This sets (by a copy) the vector parameter.
     */
    void operator=(const gsl_vector* new_theta) {
      if(new_theta == (gsl_vector*)0) 
	throw rl::exception::SARSABadParam("theta = NULL");
      else if(new_theta->size != theta->size) {
	std::ostringstream ostr;
	ostr << "argument size is " << new_theta->size 
	     << " but sould be " << theta->size << ".";
	throw rl::exception::SARSABadParam(ostr.str());
      }

      gsl_vector_memcpy(theta,new_theta);
    }

    TD(const ARCHITECTURE& a, const TD_PARAM& p)
      : rl::Serializer(),
	archi(a), 
	td_param(p),
	theta(archi.newParameterInstance()),
	grad(archi.newParameterInstance()) {
    }

    TD(const self_type& cp)
      : rl::Serializer(cp),
	archi(cp.archi), 
	td_param(cp.td_param),
	theta(0),
	grad(0) {
      paramCopy(cp.theta, cp.grad);
    }

    self_type& operator=(const self_type& cp) {
      if(this != &cp) {
	archi = cp.archi;
	td_param = cp.td_param;
	paramCopy(cp.theta, cp.grad);
      }
      return *this;
    }

    virtual ~TD(void) {
      gsl_vector_free(theta);
      gsl_vector_free(grad);
    }
                   
    output_type operator()(const input_type& input) const {
      return archi(theta,input);
    }

    template<typename TRANSITION>
    void update(const TRANSITION& transition){
      output_type v,next_v,td;

      v = archi(theta,transition.current());
      archi.gradient(theta,transition.current(),grad);

      if(transition.isTerminal())
	td = transition.reward() - v;
      else {
	next_v = archi(theta,transition.next());
	td = transition.reward() + td_param.gamma()*next_v - v;
      }
      
      // theta <- theta + alpha*td*grad
      gsl_blas_daxpy(td*td_param.alpha(),
		     grad,
		     theta);
    }

    template<typename TRANSITION_ITERATOR>
    void update(const TRANSITION_ITERATOR& begin, const TRANSITION_ITERATOR& end) {
      for(TRANSITION_ITERATOR i=begin; i!=end; ++i)
	update(*i);
    }
  };

  
  template<typename ARCHITECTURE,
	   typename TD_PARAM>
  TD<ARCHITECTURE,TD_PARAM> td(const ARCHITECTURE& a, const TD_PARAM& p) {
    return TD<ARCHITECTURE,TD_PARAM>(a,p);
  }

  namespace sa {
    /**
     * @short SARSA algorithm
     *
     */
    template<typename SA_ARCHITECTURE,
	     typename TD_PARAM>
    class SARSA : public TD<SA_ARCHITECTURE,TD_PARAM> {
    public: 
      
      typedef SARSA<SA_ARCHITECTURE,TD_PARAM>         self_type;  
      typedef TD<SA_ARCHITECTURE,TD_PARAM>            super_type;         
      typedef typename SA_ARCHITECTURE::output_type   output_type;                
      typedef typename SA_ARCHITECTURE::state_type    state_type;                   
      typedef typename SA_ARCHITECTURE::action_type   action_type; 

    public:


      SARSA(const SA_ARCHITECTURE& a, const TD_PARAM& p) : super_type(a,p) {}
      SARSA(const  self_type& cp) : super_type(cp) {}
      self_type& operator=(const self_type& cp) {
	return this->super_type::operator=(cp);
      }

      virtual ~SARSA(void) {}

      output_type operator()(const state_type& s, const action_type& a) const {
	return this->super_type::operator()(rl::sa_pair(s,a));
      }
    };

    template<typename SA_ARCHITECTURE,
	     typename TD_PARAM>
    SARSA<SA_ARCHITECTURE,TD_PARAM> sarsa(const SA_ARCHITECTURE& a, const TD_PARAM& p) {
      return SARSA<SA_ARCHITECTURE,TD_PARAM>(a,p);
    }

  }

}


#endif
