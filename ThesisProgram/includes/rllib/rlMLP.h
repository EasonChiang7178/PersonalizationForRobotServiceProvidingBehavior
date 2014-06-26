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
#ifndef rlMLP_H
#define rlMLP_H

#include <rlTypes.h>
#include <rlConcept.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

namespace rl {
  namespace architecture {
    namespace mlp {

      /**
       * @short This is the identity transfer function.
       */
      class Identity : public rl::fits<rl::concept::mlp::Transfer> {
      public:
	double operator()(double weighted_sum) const {
	  return weighted_sum;
	}
	std::string name(void) const {return "f(x) = x";}
      };
      
      inline Identity identity(void) {return Identity();}

      /**
       * @short This acts as a sigmoid, since it is y=ax kept in [-1,1]
       */
      template<typename SATURATION_PARAM>
      class Saturation  : public rl::fits<rl::concept::mlp::Transfer> {
      private:

	const SATURATION_PARAM& param;
	
      public:

	Saturation(const SATURATION_PARAM& p) : param(p) {}
	Saturation(const Saturation<SATURATION_PARAM>& cp) : param(cp.param) {}

	double operator()(double weighted_sum) const {
	  double res = weighted_sum*param.a();
	  if(res > 1)
	    return 1;
	  if(res < -1)
	    return -1;
	  
	  return weighted_sum;
	}

	std::string name(void) const {
	  std::ostringstream ostr;
	  ostr << "f(x) = " << param.a() << " * x bounded in [0,1]";
	  return ostr.str();
	}
      };

      template<typename SATURATION_PARAM>
      Saturation<SATURATION_PARAM> saturation(const SATURATION_PARAM& p) {
	return Saturation<SATURATION_PARAM>(p);
      }
      
      /**
       * @short This tanh(ax) saturation.
       */
      template<typename SATURATION_PARAM>
      class Tanh {
      private:

	const SATURATION_PARAM& param;

      public:

	Tanh(const SATURATION_PARAM& p) : param(p) {}
	Tanh(const Tanh<SATURATION_PARAM>& cp) : param(cp.param) {}

	double operator()(double weighted_sum) const {
	  return tanh(weighted_sum*param.a());
	}
	
	std::string name(void) const {
	  std::ostringstream ostr;
	  ostr << "f(x) = tanh(" << param.a() << " * x)";
	  return ostr.str();
	}
      };

      template<typename SATURATION_PARAM>
      Tanh<SATURATION_PARAM> tanh(const SATURATION_PARAM& p) {
	return Tanh<SATURATION_PARAM>(p);
      }


      /**
       * @short This defines the input layer of the neural network.
       */
      template<typename FEATURE> 
      class Input {
      private:

	gsl_vector* xx;
	const FEATURE& phi;

      public:
	typedef FEATURE                           feature_type;
	typedef typename feature_type::input_type input_type;

	int rank(void)         const {return 0;}
	int minParamRank(void) const {return 0;}
	int nbParams(void)     const {return 0;}
	int layerSize(void)    const {return phi.dimension();}

	void displayParameters(std::ostream& os) const {
	  os << "Input  #" << std::setw(3) << rank()
	     << " :        no weight"
	     << " : size = " << std::setw(4) << layerSize() << std::endl;
	}

	Input(const FEATURE& f) 
	  : xx(0),
	    phi(f) {
	  xx = gsl_vector_alloc(layerSize());
	}
	Input(const Input<FEATURE>& cp) : xx(0), phi(cp.phi) {
	  xx = gsl_vector_alloc(cp.xx->size);
	  gsl_vector_memcpy(xx,cp.xx);
	}

	Input<FEATURE> operator=(const Input<FEATURE>& cp) {
	  if(this != &cp) {
	    phi = cp.phi;
	    gsl_vector_memcpy(xx,cp.xx);
	  }
	  return this;
	}

	~Input(void) {gsl_vector_free(xx);}

	void operator()(const gsl_vector* theta, const input_type& x,
			double* y) const {
	  unsigned int i;
	  phi(x,xx);
	  double* end = y+layerSize();
	  double* iter;
	  for(iter = y, i=0; iter != end; ++i,++iter) 
	    *iter = gsl_vector_get(xx,i);
	}
      };

      template<typename FEATURE> 
      Input<FEATURE> input(const FEATURE& f) {
	return Input<FEATURE>(f);
      }

      /**
       * @short This defines the some hidden layer of the neural network.
       */
      template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
      class Hidden {
      private:
	int size;
	std::vector<double> yy;

      public:
	typedef typename PREVIOUS_LAYER::input_type           input_type;

	PREVIOUS_LAYER& input;
	const MLP_TRANSFER& f;
	
	int rank(void)         const {return 1+input.rank();}
	int minParamRank(void) const {return input.minParamRank()+input.nbParams();}
	int nbParams(void)     const {return size*(1+input.layerSize());}
	int layerSize(void)    const {return size;}


	void displayParameters(std::ostream& os) const {
	  input.displayParameters(os);
	  os << "Input  #" << std::setw(3) << rank()
	     << " : " << "[" << std::setw(6) << minParamRank() << ", " << std::setw(6) << minParamRank()+nbParams() << "["
	     << " : size = " << std::setw(4) << layerSize()
	     << " : " << f.name() << std::endl;
	}

	Hidden(PREVIOUS_LAYER& in,
	       int layer_size,
	       const MLP_TRANSFER& transfer) : size(layer_size), yy(), input(in), f(transfer) {}
	Hidden(const Hidden<PREVIOUS_LAYER,MLP_TRANSFER>& cp) 
	  : size(cp.size), yy(cp.yy), input(cp.input), f(cp.f) {}
	Hidden<PREVIOUS_LAYER,MLP_TRANSFER>& operator=(const Hidden<PREVIOUS_LAYER,MLP_TRANSFER>& cp) {
	  if(this != &cp) {
	    size = cp.size; 
	    yy = cp.yy; 
	    input = cp.input;
	    f = cp.f;
	  }
	  return *this;
	}

	void operator()(const gsl_vector* theta,
			const input_type& x,
			double* y) const {
	  unsigned int k;
	  double sum;

	  std::vector<double>& _yy = const_cast<std::vector<double>&>(yy);
	  _yy.resize(input.layerSize());

	  input(theta,x,&(*(_yy.begin())));

	  typename std::vector<double>::const_iterator j,yyend;
	  double* i;
	  double* yend = y+layerSize();

	  k=minParamRank();
	  yyend = yy.end();
	  for(i=y;i!=yend;++i) {
	    sum = gsl_vector_get(theta,k);++k;
	    for(j=yy.begin();j!=yyend;++j,++k)
	      sum += gsl_vector_get(theta,k)*(*j);
	    *i = f(sum);
	  }
	}
      };

      template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
      Hidden<PREVIOUS_LAYER,MLP_TRANSFER> hidden(PREVIOUS_LAYER& in,
						 int layer_size,
						 const MLP_TRANSFER& transfer) {
	return Hidden<PREVIOUS_LAYER,MLP_TRANSFER>(in,layer_size,transfer);
      }

      /**
       * @short This defines the output layer of the neural network.
       */
      template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
      class Output {
      private:
	std::vector<double> y;
      public:

	typedef typename PREVIOUS_LAYER::input_type        input_type;
	typedef double                                    output_type;
	typedef gsl_vector* 	                          param_type;
	
	int rank(void)         const {return 1+input.rank();}
	int minParamRank(void) const {return input.minParamRank()+input.nbParams();}
	int nbParams(void)     const {return 1*(1+input.layerSize());}
	int layerSize(void)    const {return 1;}
	
	PREVIOUS_LAYER& input;
	const MLP_TRANSFER& f;
	

	void displayParameters(std::ostream& os) const {
	  input.displayParameters(os);
	  os << "Input  #" << std::setw(3) << rank()
	     << " : " << "[" << std::setw(6) << minParamRank() << ", " << std::setw(6) << minParamRank()+nbParams() << "["
	     << " : size = " << std::setw(4) << layerSize()
	     << " : " << f.name() << std::endl;
	}

	Output(PREVIOUS_LAYER& in,
	       const MLP_TRANSFER& transfer) 
	  : y(),input(in), f(transfer) {}
	Output(const Output<PREVIOUS_LAYER,MLP_TRANSFER>& cp)
	  : y(cp.y), input(cp.input), f(cp.f) {}
	Output<PREVIOUS_LAYER,MLP_TRANSFER>& operator=(const Output<PREVIOUS_LAYER,MLP_TRANSFER>& cp) {
	  if(this != &cp) {
	    y = cp.y; 
	    input = cp.input;
	    f = cp.f;
	  }
	  return *this;
	}


	param_type newParameterInstance (void) const {return gsl_vector_calloc(minParamRank()+nbParams());}

	double operator()(const gsl_vector* theta, const input_type& x) const {
	  unsigned int k;
	  double sum;
	  
	  std::vector<double>& _y = const_cast<std::vector<double>&>(y);
	  _y.resize(input.layerSize());

	  input(theta,x,&(*(_y.begin())));

	  typename std::vector<double>::const_iterator j,yend;

	  k=minParamRank();
	  yend = y.end();
	  sum = gsl_vector_get(theta,k);++k;
	  for(j=y.begin();j!=yend;++j,++k)
	    sum += gsl_vector_get(theta,k)*(*j);
	  return f(sum);
	}
      };

      template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
      Output<PREVIOUS_LAYER,MLP_TRANSFER> output(PREVIOUS_LAYER& in,
						 const MLP_TRANSFER& transfer) {
	return Output<PREVIOUS_LAYER,MLP_TRANSFER>(in,transfer);
      }

      namespace sa {
	template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
	class Output 
	  : public mlp::Output<PREVIOUS_LAYER,MLP_TRANSFER> {
	public:

	  typedef mlp::Output<PREVIOUS_LAYER,MLP_TRANSFER>  super_type;
	  typedef typename PREVIOUS_LAYER::input_type       input_type;
	  typedef typename input_type::state_type           state_type;
	  typedef typename input_type::action_type          action_type;
	  typedef double                                    output_type;
	  typedef gsl_vector* 	                            param_type;
	  

	  Output(PREVIOUS_LAYER& in,
		 const MLP_TRANSFER& transfer) : super_type(in,transfer) {}
	  Output(const Output<PREVIOUS_LAYER,MLP_TRANSFER>& cp) : super_type(cp) {}
	  
	  double operator()(const gsl_vector* theta, const state_type& s, const action_type& a) const {
	    return this->super_type::operator()(theta,rl::sa_pair(s,a));
	  }
	};

	template<typename PREVIOUS_LAYER,typename MLP_TRANSFER>
	Output<PREVIOUS_LAYER,MLP_TRANSFER> output(PREVIOUS_LAYER& in,
						   const MLP_TRANSFER& transfer) {
	  return Output<PREVIOUS_LAYER,MLP_TRANSFER>(in,transfer);
	}
      }
    }
  }
}

#endif
