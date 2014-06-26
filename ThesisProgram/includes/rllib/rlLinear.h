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
#ifndef rlLINEAR_H
#define rlLINEAR_H

#include <rlTypes.h>
#include <rlConcept.h>

namespace rl {
  namespace architecture {
    namespace linear {

      /**
       * @short This Builds an architecture from a feature. 
       *
       */
      template<typename FEATURE>
      class DotProduct {
      private:
    
	
	gsl_vector* tmp;
	const FEATURE& phi;

      public:
    
	typedef double                       output_type;
	typedef typename FEATURE::input_type input_type;
	typedef gsl_vector*                  param_type;
    
    
	DotProduct(const FEATURE& feature) 
	  : tmp(gsl_vector_calloc(feature.dimension())), phi(feature) {}
	DotProduct(const DotProduct& cp) 
	  : tmp(gsl_vector_calloc(cp.phi.dimension())), phi(cp.phi) {}
	virtual ~DotProduct(void) {
	  gsl_vector_free(tmp);
	}
    
	output_type operator()(const param_type& theta, 
			       const input_type& x) const {
	  double res;

	  phi(x,tmp);
	  gsl_blas_ddot(theta,tmp,&res);
	  return res;
	}
    
	param_type newParameterInstance(void) const {
	  return gsl_vector_calloc(phi.dimension());
	}

	void gradient(const param_type& theta, 
		      const input_type& x, 
		      param_type& grad) const {

	  if(theta == (param_type)0)
	    throw rl::exception::NullVectorPtr("in rl::DotProduct::gradient");
	  else if((int)(grad->size) != phi.dimension())
	    throw rl::exception::BadVectorSize(grad->size,phi.dimension(),
					       "in rl::DotProduct::gradient");
	  phi(x,grad);
	}
      };

      template<typename FEATURE>
      DotProduct<FEATURE> dot_product(const FEATURE& feature) {
	return DotProduct<FEATURE>(feature);
      }

      namespace sa {
	/**
	 * @short This Builds an architecture from a feature. 
	 *
	 */
	template<typename SA_FEATURE>
	class DotProduct 
	  : public linear::DotProduct<SA_FEATURE> {
	public:
	  typedef linear::DotProduct<SA_FEATURE>        super_type;
	  typedef typename super_type::input_type       input_type;
	  typedef typename super_type::output_type      output_type;
	  typedef typename super_type::param_type       param_type;
	  typedef typename input_type::state_type       state_type;
	  typedef typename input_type::action_type      action_type;
       
	  DotProduct(const SA_FEATURE& feature) : super_type(feature) {}
	  DotProduct(const DotProduct<SA_FEATURE>& cp) : super_type(cp) {}
	  virtual ~DotProduct(void) {}

	  output_type operator()(const param_type theta, 
				 const input_type& x) const {
	    return this->super_type::operator()(theta,x);
	  }

	  void gradient(const param_type& theta, 
			const input_type& x,
			param_type& grad) const {
	    this->super_type::gradient(theta,x,grad);
	  }
      
	  output_type operator()(const param_type theta, 
				 const state_type& s, 
				 const action_type& a) const {
	    return (*this)(theta,rl::sa_pair(s,a));
	  }

	  void gradient(const param_type& theta, 
			const state_type& s, 
			const action_type& a, 
			param_type& grad) const {
	    (*this).gradient(theta,rl::sa_pair(s,a),grad);
	  }
	};
	template<typename SA_FEATURE>
	DotProduct<SA_FEATURE> dot_product(const SA_FEATURE& feature) {
	  return DotProduct<SA_FEATURE>(feature);
	}
      }
    }
  }
}


#endif
