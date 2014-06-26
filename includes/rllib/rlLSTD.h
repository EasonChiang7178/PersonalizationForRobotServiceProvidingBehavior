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

#ifndef rlLSTD_H
#define rlLSTD_H

#include <rlException.h>
#include <rlTypes.h>
#include <rlTransition.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <iostream>


namespace rl {

  namespace default_param {
    template<typename GAMMA_PARAM>
    class LSTD : public GAMMA_PARAM {
    public:
      double reg(void) const {return 0;}
    };
  }
  
  

  /**
   * @short LSTD algorithm. It  fits rl::BatchCritic
   */
  template<typename ARCHITECTURE,
	   typename LSTD_PARAM>
  class LSTD 
    : public rl::Serializer {

  public:

    typedef LSTD<ARCHITECTURE,LSTD_PARAM>      self_type;
    typedef typename ARCHITECTURE::input_type  input_type;
    typedef typename ARCHITECTURE::output_type output_type;
    typedef typename ARCHITECTURE::param_type  param_type;

  protected:
    
    const ARCHITECTURE& archi;
    const LSTD_PARAM&   param;
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

    LSTD(const ARCHITECTURE& a, const LSTD_PARAM& p)
      : archi(a),param(p),
	theta(archi.newParameterInstance()),
	grad(archi.newParameterInstance()) {}
    LSTD(const self_type& cp) 
      : archi(cp.archi), param(cp.param),
	theta(0), grad(0) {
      paramCopy(cp.theta, cp.grad);
    }

    self_type& operator=(const self_type& cp) {
      if(this != &cp) {
	archi = cp.archi;
	param = cp.param;
	paramCopy(cp.theta, cp.grad);
      }
      return *this;
    }

    ~LSTD(void) {
      gsl_vector_free(theta);
      gsl_vector_free(grad);
    }

    /**
     * Get the architecture parameter computed by TD.
     */
    const gsl_vector* parameter(void) const {
      return theta;
    }

    output_type operator()(const input_type& x) const {
      return archi(theta,x);
    }

    template<typename TRANSITION_ITERATOR>
    void update(const TRANSITION_ITERATOR& begin, const TRANSITION_ITERATOR& end) {
      const typename TRANSITION_ITERATOR::value_type *t;
      int n         = theta->size;
      int signum;
      gsl_matrix *M = gsl_matrix_calloc(n, n);
      gsl_vector *b = gsl_vector_calloc(n);
      gsl_vector *tmp1 = gsl_vector_calloc(n);
      gsl_vector *tmp2 = gsl_vector_calloc(n);
      gsl_permutation * p = gsl_permutation_alloc (n);

      gsl_matrix_set_identity(M);
      gsl_matrix_scale(M,param.reg());

      for(TRANSITION_ITERATOR i=begin; i!=end; ++i) {
	t = &(*i);
	archi.gradient(theta,t->current(),tmp1);
	gsl_blas_dger(1, tmp1, tmp1, M);
	if(!t->isTerminal()) {
	  archi.gradient(theta,t->next(),tmp2);
	  gsl_blas_dger(-param.gamma(), tmp1, tmp2, M);
	}
	gsl_blas_daxpy(t->reward(), tmp1, b); 
      }
      
      // Inversion of M
      gsl_linalg_LU_decomp (M, p, &signum);
      gsl_linalg_LU_solve (M, p, b, theta);
     
      gsl_vector_free(tmp2);
      gsl_vector_free(tmp1);	
      gsl_permutation_free (p);
      gsl_matrix_free(M);
      gsl_vector_free(b);
    }
  };

  template<typename ARCHITECTURE,
	   typename LSTD_PARAM>
  LSTD<ARCHITECTURE,LSTD_PARAM> lstd(const ARCHITECTURE& a, const LSTD_PARAM& p) {
    return LSTD<ARCHITECTURE,LSTD_PARAM>(a,p);
  }

  namespace sa {
    /**
     * SA version of LSTD.
     */
    template<typename SA_ARCHITECTURE,
	     typename LSTD_PARAM>
    class LSTD 
      : public rl::LSTD<SA_ARCHITECTURE,LSTD_PARAM> {
    public:

      typedef LSTD<SA_ARCHITECTURE,LSTD_PARAM>       self_type;
      typedef rl::LSTD<SA_ARCHITECTURE,LSTD_PARAM>   super_type;
      typedef typename SA_ARCHITECTURE::input_type   input_type;
      typedef typename SA_ARCHITECTURE::state_type   state_type;
      typedef typename SA_ARCHITECTURE::action_type  action_type;
      typedef typename SA_ARCHITECTURE::output_type  output_type;
      
      LSTD(const SA_ARCHITECTURE& a, const LSTD_PARAM& p) : super_type(a,p) {}
      LSTD(const self_type& cp) : super_type(cp) {}
      
      self_type& operator=(const self_type& cp) {
	return this->super_type::operator=(cp);
      }

      output_type operator()(const state_type&  s,
			     const action_type& a) const {
	return this->archi(this->theta,s,a);
      }
    };

    template<typename SA_ARCHITECTURE,
	     typename LSTD_PARAM>
    LSTD<SA_ARCHITECTURE,LSTD_PARAM> lstd(const SA_ARCHITECTURE& a, const LSTD_PARAM& p) {
      return LSTD<SA_ARCHITECTURE,LSTD_PARAM>(a,p);
    }
  }
  
}

#endif
