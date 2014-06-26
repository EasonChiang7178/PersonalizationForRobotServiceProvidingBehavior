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


#ifndef rlKTD_H
#define rlKTD_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <cmath>

#include <rlException.h>
#include <rlRandom.h>

namespace rl {

  namespace default_param {
    template<typename GAMMA_PARAM>
    class KTD : public GAMMA_PARAM {
    public:  

      double eta_noise(void)          const {return    0;}
      double observation_noise(void)  const {return    1;}
      double priorVar(void)           const {return   10;}
      double randomAmplitude(void)    const {return    0;}
      double ut_alpha(void)           const {return 1e-1;}
      double ut_beta(void)            const {return    2;}
      double ut_kappa(void)           const {return    0;}

      /**
       * It toggles a faster evaluation of the SA_ARCHITECTURE. It is relevant only for linear architectures.
       */
      bool   use_linear_evaluation(void)  {return false;}
    };
  }

  namespace sa {
    /**
     * @short Common class for KTD algorithms
     *
     * ACTION_ITERATOR Used for the argmax function.<br>
     */
    template<typename SA_ARCHITECTURE,
	     typename ACTION_ITERATOR,
	     typename KTD_PARAM>
    class KTD : public rl::Serializer {
    public: 
      typedef typename SA_ARCHITECTURE::input_type    input_type;        
      typedef typename SA_ARCHITECTURE::output_type   output_type;             
      typedef typename SA_ARCHITECTURE::state_type    state_type;  
      typedef typename SA_ARCHITECTURE::action_type   action_type;
      typedef gsl_vector*                             param_type;  

      typedef KTD<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM> self_type;
      
    protected:
    
      const SA_ARCHITECTURE& archi;
      const ACTION_ITERATOR& aiter;
      const KTD_PARAM&       param;

      gsl_vector* theta;
      unsigned int theta_size;
      gsl_matrix* sigmaTheta;
      gsl_matrix* sigmaPointsSet;
      gsl_matrix *U;
      gsl_vector *D;
      gsl_vector *y;
      gsl_vector *ktdQ_images_SP;
      gsl_vector *P_theta_r;
      gsl_vector *kalmanGain;
      gsl_vector *centeredSP;
      
      double w_m0;
      double w_c0;
      double w_i;
      double lambdaUt;
      
		
    protected:

	
      virtual void read(std::istream& is) {
	is >> w_m0 
	   >> w_c0
	   >> w_i
	   >> theta
	   >> sigmaTheta
	   >> sigmaPointsSet;
      }

      virtual void write(std::ostream& os) const {
	os << w_m0 << ' '
	   << w_c0 << ' '
	   << w_i << ' '
	   << theta
	   << sigmaTheta
	   << sigmaPointsSet;
      }

    private:

      void initWeights(void) {
	double alphaUt = param.ut_alpha();
	double betaUt = param.ut_beta();
	double kappaUt = param.ut_kappa();

	lambdaUt = alphaUt*alphaUt*(theta_size+kappaUt) - theta_size ;
	w_m0 = lambdaUt/(theta_size+lambdaUt) ;
	w_c0 = w_m0 + 1 - alphaUt*alphaUt + betaUt ;
	w_i = 1.0/(2*(theta_size+lambdaUt)) ;
      }


      void centralDifferencesTransform(void){
	  	  
	// allocate size L  
	unsigned int i;
	gsl_vector_view this_sigmaPoint;
	gsl_vector_view that_sigmaPoint;
	  
	// Then the sigma points can be computed
	// first i=0;
	this_sigmaPoint = gsl_matrix_column(sigmaPointsSet,0);
	gsl_vector_memcpy(&(this_sigmaPoint.vector), theta);
	// the 1<= i <= L
	for(i=1;i<theta_size+1; i++){
	  this_sigmaPoint = gsl_matrix_column(sigmaPointsSet,i);
	  gsl_vector_memcpy(&(this_sigmaPoint.vector), theta);
	  that_sigmaPoint = gsl_matrix_column(sigmaTheta,i-1);
	  gsl_blas_daxpy(sqrt(theta_size+lambdaUt), &(that_sigmaPoint.vector), &(this_sigmaPoint.vector) );
	}
	// the L+1<= i <= 2*L
	for(i=theta_size+1;i<2*theta_size+1; i++){
	  this_sigmaPoint = gsl_matrix_column(sigmaPointsSet,i);
	  gsl_vector_memcpy(&(this_sigmaPoint.vector), theta);
	  that_sigmaPoint = gsl_matrix_column(sigmaTheta,i-1-theta_size);
	  gsl_blas_daxpy(-sqrt(theta_size+lambdaUt) , &(that_sigmaPoint.vector), &(this_sigmaPoint.vector) );
	}
      }
	
      void choleskyUpdate(double alpha, gsl_vector *x){
	  
	/*
	  This function performs a cholesky update of the cholesky factorization sigmaTheta, that is it replaces 
	  the Cholesky factorization sigmaTheta by the cholesky factorization of 
	  sigmaTheta*sigmaTheta^T - alpha * x * x^T
	    
	  The algorithm is an adaptation of a LU factorization rank one update. Reference is :
	  Peter Strange, Andreas Griewank and Matthias Bollhöfer.
	  On the Efficient Update of Rectangular LU Factorizations subject to Low Rank Modifications.
	  Electronic Transactions on Numerical Analysis, 26:161-177, 2007.
	  alg. is given in left part of fig.2.1. 
	    
	  Perhaps a more efficient algorithm exists, however it should do the work for now. And the code is probably not optimal...
	    
	  WARNING
	*/
	  
	unsigned int i,j;
	double tmp;

	
			
	// A first thing is to set SS' (chol factor) in a LU form, L being unitriangular
	// Compute U = L^T and D = diag(L)
	gsl_matrix_set_zero(U);
	for(i=0; i<theta_size; i++){
	  gsl_vector_set(D,i,gsl_matrix_get(sigmaTheta,i,i));
	  for(j=0; j<=i; j++){
	    gsl_matrix_set(U,j,i,gsl_matrix_get(sigmaTheta,i,j));
	  }
	}
	// Replace L by L*D^{-1} and U by D*U
	for(i=0; i<theta_size; i++){
	  for(j=0; j<=i; j++){
	    tmp = gsl_matrix_get(sigmaTheta,i,j);
	    tmp /= gsl_vector_get(D,j);
	    gsl_matrix_set(sigmaTheta,i,j,tmp);
	    tmp = gsl_matrix_get(U,j,i);
	    tmp *= gsl_vector_get(D,j);
	    gsl_matrix_set(U,j,i,tmp);
	  }
	}
	  
	// compute the y = alpha x vector
	gsl_vector_memcpy(y,x);
	gsl_vector_scale(y,alpha);
	  
	// perform the rank 1 LU modification
	for(i=0; i<theta_size; i++){
	    
	  // diagonal update 
	  tmp = gsl_matrix_get(U,i,i) + gsl_vector_get(x,i)*gsl_vector_get(y,i);
	  gsl_matrix_set(U,i,i,tmp);
	  tmp = gsl_vector_get(y,i);
	  tmp /= gsl_matrix_get(U,i,i);
	  gsl_vector_set(y,i,tmp);
	    
	  for(j=i+1; j<theta_size; j++){
	    // L (that is sigmaTheta) update 
	    tmp = gsl_vector_get(x,j) - gsl_vector_get(x,i)*gsl_matrix_get(sigmaTheta,j,i);
	    gsl_vector_set(x,j,tmp);
	    tmp = gsl_matrix_get(sigmaTheta,j,i) + gsl_vector_get(y,i) * gsl_vector_get(x,j);
	    gsl_matrix_set(sigmaTheta,j,i,tmp);
	  }
	    
	  for(j=i+1; j<theta_size; j++){
	    // U update 
	    tmp = gsl_matrix_get(U,i,j) + gsl_vector_get(x,i)*gsl_vector_get(y,j);
	    gsl_matrix_set(U,i,j,tmp);
	    tmp = gsl_vector_get(y,j) - gsl_vector_get(y,i) * gsl_matrix_get(U,i,j);
	    gsl_vector_set(y,j,tmp);
	  }
	}
	  
	// Now we want the chol decomposition
	// first D = sqrt(diag(U));
	for(i=0; i<theta_size; i++){
	  tmp =  gsl_matrix_get(U,i,i);
	  if(tmp<=0)
	    throw exception::NotPositiveDefiniteMatrix("in ..::KTD::choleskyUpdate");
	  gsl_vector_set(D,i,sqrt(tmp));
	}
	// then L = L*D;
	for(i=0; i<theta_size; i++){
	  for(j=0; j<theta_size; j++){
	    tmp = gsl_matrix_get(sigmaTheta,i,j) * gsl_vector_get(D,j);
	    gsl_matrix_set(sigmaTheta,i,j,tmp);
	  }
	}
	// that's all folks !
      }


      virtual action_type selectAction(const input_type& next,
				       output_type& vValue,
				       unsigned int i) const=0;

      template<typename TRANSITION>
      void kalmanUpdate(const TRANSITION& transition){
	  
	// initializations
	unsigned int i;
	double d, P_r, pred_r;
	output_type qValue, vValue;
	// action_type a;
	const state_type&  state     = transition.current().s;
	const action_type& action    = transition.current().a; 
	output_type        reward    = transition.reward();
	gsl_vector         sigmaPoint; /* Not a pointer ! */
	
	/*
	  -------Prediction Step ----------------------------------------------------
	*/
	  
	// nothing to do for thetaPred
	gsl_matrix_scale(sigmaTheta,sqrt(1+param.eta_noise()));
	  
	/*
	  -------Compute the sigma-Points and their images ---------------------------
	*/
	  
	// compute the sigma-points (weights are initialized at the creation of the agent)
	centralDifferencesTransform();
	  
	//compute their images
	if(transition.isTerminal())
	  for(i=0; i<2*theta_size+1; i++){
	    sigmaPoint = gsl_matrix_column(sigmaPointsSet,i).vector;
	    qValue = archi(&sigmaPoint,
			   state,action);
	    gsl_vector_set(ktdQ_images_SP,i,(double)(qValue));
	  }
	else {
	  for(i=0; i<2*theta_size+1; i++){
	    sigmaPoint = gsl_matrix_column(sigmaPointsSet,i).vector;
	    qValue = archi(&sigmaPoint,
			   state,action);
	    /*a      = */ selectAction(transition.next(),
				       vValue,i);
	    gsl_vector_set(ktdQ_images_SP,i,
			   (double)(qValue - param.gamma() * vValue));
	  }
	}
	  
	  
	/*
	  -------Compute Statistics of interest --------------------------------------
	*/
	  
	// predicted reward
	pred_r = w_m0 * gsl_vector_get(ktdQ_images_SP,0) ;
	for(i=1;i<2*theta_size+1; i++){
	  pred_r += w_i * gsl_vector_get(ktdQ_images_SP,i);
	}

	 	  
	// associated variance(s)
	d = gsl_vector_get(ktdQ_images_SP,0) - pred_r ;
	P_r = w_c0 * d * d  ;
	for(i=1; i<2*theta_size+1; i++){
	  // reward
	  d = gsl_vector_get(ktdQ_images_SP,i) - pred_r ;
	  P_r += w_i * d * d ;
	}
	P_r += param.observation_noise();

	// Correlation between parameters and reward
	gsl_vector_set_zero(P_theta_r) ;
	for(i=1; i<2*theta_size+1; i++){
	  sigmaPoint = gsl_matrix_column(sigmaPointsSet,i).vector;
	  gsl_vector_memcpy(centeredSP, &sigmaPoint) ;
	  gsl_vector_sub(centeredSP,theta) ;
	  gsl_blas_daxpy(w_i * (gsl_vector_get(ktdQ_images_SP,i) - pred_r), centeredSP, P_theta_r) ;
	}

	/*
	  -------Correction equations --------------------------------------
	*/
	  
	// kalman gain
	gsl_vector_memcpy(kalmanGain, P_theta_r);
	gsl_vector_scale(kalmanGain, 1.0/P_r);
	  
	// Update mean
	gsl_blas_daxpy(reward - pred_r, kalmanGain, theta);
	  
	// Update Covariance
	choleskyUpdate(-P_r,kalmanGain);
      }


      void paramCopy(const gsl_vector* theta_,
		     const gsl_matrix* sigmaTheta_,
		     const gsl_matrix* sigmaPointsSet_,
		     const gsl_matrix* U_,
		     const gsl_vector* y_,
		     const gsl_vector* D_,
		     const gsl_vector* ktdQ_images_SP_,
		     const gsl_vector* P_theta_r_,
		     const gsl_vector* kalmanGain_,
		     const gsl_vector* centeredSP_) {

	gsl_vector_free(theta);
	gsl_matrix_free(sigmaTheta);
	gsl_matrix_free(sigmaPointsSet);
	gsl_matrix_free(U);
	gsl_vector_free(y);
	gsl_vector_free(D);
	gsl_vector_free(ktdQ_images_SP);
	gsl_vector_free(P_theta_r);
	gsl_vector_free(kalmanGain);
	gsl_vector_free(centeredSP);

	theta = 0;
	sigmaTheta = 0;
	sigmaPointsSet = 0;
	U = 0;
	y = 0;
	D = 0;
	ktdQ_images_SP = 0;
	P_theta_r = 0;
	kalmanGain = 0;
	centeredSP = 0;


	if(theta_ != 0) {
	  theta = gsl_vector_alloc(theta_->size);
	  gsl_vector_memcpy(theta, theta_);
	}

	if(sigmaTheta_ != 0) {
	  sigmaTheta = gsl_matrix_alloc(sigmaTheta_->size1,
					sigmaTheta_->size2);
	  gsl_matrix_memcpy(sigmaTheta, sigmaTheta_);
	}

	if(sigmaPointsSet_ != 0) {
	  sigmaPointsSet = gsl_matrix_alloc(sigmaPointsSet_->size1,
					    sigmaPointsSet_->size2);
	  gsl_matrix_memcpy(sigmaPointsSet, sigmaPointsSet_);
	}

	if(U_ != 0) {
	  U = gsl_matrix_alloc(U_->size1,
			       U_->size2);
	  gsl_matrix_memcpy(U, U_);
	}

	if(y_ != 0) {
	  y = gsl_vector_alloc(y_->size);
	  gsl_vector_memcpy(y, y_);
	}

	if(D_ != 0) {
	  D = gsl_vector_alloc(D_->size);
	  gsl_vector_memcpy(D, D_);
	}

	if(ktdQ_images_SP_ != 0) {
	  ktdQ_images_SP = gsl_vector_alloc(ktdQ_images_SP_->size);
	  gsl_vector_memcpy(ktdQ_images_SP, ktdQ_images_SP_);
	}

	if(P_theta_r_ != 0) {
	  P_theta_r = gsl_vector_alloc(P_theta_r_->size);
	  gsl_vector_memcpy(P_theta_r, P_theta_r_);
	}

	if(kalmanGain_ != 0) {
	  kalmanGain = gsl_vector_alloc(kalmanGain_->size);
	  gsl_vector_memcpy(kalmanGain, kalmanGain_);
	}

	if(centeredSP_ != 0) {
	  centeredSP = gsl_vector_alloc(centeredSP_->size);
	  gsl_vector_memcpy(centeredSP, centeredSP_);
	}
    }

    public:

      KTD(const SA_ARCHITECTURE& a,
	  const ACTION_ITERATOR& it,
	  const KTD_PARAM& p) 
	: archi(a), aiter(it), param(p),
	  theta(archi.newParameterInstance()),
	  theta_size(theta->size),
	  sigmaTheta(gsl_matrix_alloc(theta_size,theta_size)),
	  sigmaPointsSet(gsl_matrix_calloc(theta_size,2*theta_size+1)),
	  U(gsl_matrix_alloc(theta_size,theta_size)),
	  D(gsl_vector_alloc(theta_size)),
	  y(gsl_vector_alloc(theta_size)),
	  ktdQ_images_SP(gsl_vector_alloc(2*theta_size + 1)),
	  P_theta_r(gsl_vector_alloc(theta_size)),
	  kalmanGain(gsl_vector_alloc(theta_size)),
	  centeredSP(gsl_vector_alloc(theta_size)) {

	for(unsigned int i=0;i<theta_size;++i)
	  gsl_vector_set(theta,i,param.randomAmplitude()*rl::random::toss(-1,1));
	gsl_matrix_set_identity(sigmaTheta);
	gsl_matrix_scale(sigmaTheta,param.priorVar());
	initWeights();
      }

      KTD(const self_type& cp) 
	: archi(cp.archi), aiter(cp.aiter), param(cp.param),
	  theta(0),
	  theta_size(cp.theta_size),
	  sigmaTheta(0),
	  sigmaPointsSet(0),
	  U(0),
	  D(0),
	  y(0),
	  ktdQ_images_SP(0),
	  P_theta_r(0),
	  kalmanGain(0),
	  centeredSP(0) {
	paramCopy(cp.theta,
		  cp.sigmaTheta,
		  cp.sigmaPointsSet,
		  cp.U,
		  cp.y,
		  cp.D,
		  cp.ktdQ_images_SP,
		  cp.P_theta_r,
		  cp.kalmanGain,
		  cp.centeredSP);
      }
	  
    self_type& operator=(const self_type& cp) {
      if(this != &cp) {
	archi      = cp.archi;
	aiter      = cp.aiter;
	param      = cp.param;
	theta_size = cp.theta_size;
	paramCopy(cp.theta,
		  cp.sigmaTheta,
		  cp.sigmaPointsSet,
		  cp.U,
		  cp.y,
		  cp.D,
		  cp.ktdQ_images_SP,
		  cp.P_theta_r,
		  cp.kalmanGain,
		  cp.centeredSP);
      }
      return *this;
    }
      
      virtual ~KTD(void) {
	gsl_vector_free(theta);
	gsl_matrix_free(sigmaTheta);
	gsl_matrix_free(sigmaPointsSet);
	gsl_matrix_free(U);
	gsl_vector_free(y);
	gsl_vector_free(D);
	gsl_vector_free(ktdQ_images_SP);
	gsl_vector_free(P_theta_r);
	gsl_vector_free(kalmanGain);
	gsl_vector_free(centeredSP);
      }

      output_type operator()(const input_type &sa) const {
	return (*this)(sa.s,sa.a);
      }

      output_type operator()(const input_type &sa, double& variance) const {
	return (*this)(sa.s,sa.a,variance);
      }

    public:

      output_type operator()(const state_type &s, const action_type &a) const {
	unsigned int i;
	output_type pred_r;
	output_type  qval;
	gsl_vector sigmaPoint; /* not a pointer ! */
	
	if(param.use_linear_evaluation())
	  pred_r = archi(theta,s,a);
	else {
	  for(i=0; i<2*theta_size+1; i++){
	    sigmaPoint = gsl_matrix_column(sigmaPointsSet,i).vector;
	    qval = archi(&sigmaPoint,s,a);
	    gsl_vector_set(ktdQ_images_SP,i,(double)(qval));
	  }
	  
	  pred_r = w_m0 * gsl_vector_get(ktdQ_images_SP,0);
	  for(i=1;i<2*theta_size+1; i++){
	    pred_r += w_i * gsl_vector_get(ktdQ_images_SP,i);
	  }
	}
	
	
	return pred_r;
      }

      output_type operator()(const state_type &s, const action_type &a, double& variance) const {
	unsigned int i;
	output_type pred_r;
	double d;
	  
	pred_r = (*this)(s,a);

	d = gsl_vector_get(ktdQ_images_SP,0) - pred_r ;
	variance = w_c0 * d * d  ;
	for(i=1; i<2*theta_size+1; i++){
	  // reward
	  d = gsl_vector_get(ktdQ_images_SP,i) - pred_r ;
	  variance += w_i * d * d ;
	}

	return pred_r;
      }

      template<typename TRANSITION>
      void update(const TRANSITION& transition){
	kalmanUpdate(transition);
      } 

      template<typename TRANSITION_ITERATOR>
      void update(const TRANSITION_ITERATOR& begin, const TRANSITION_ITERATOR& end) {
	for(TRANSITION_ITERATOR i=begin; i!=end; ++i)
	update(*i);
    }


      action_type argmax(const state_type &s, output_type &max) const {
	action_type  a(aiter.first());
	action_type  a_max(a);
	output_type  qval;

	max = (*this)(s,a);
	while(aiter.hasNext(a)) {
	  a = aiter.next(a);
	  if((qval=(*this)(s,a))>max) {
	    a_max = a;
	    max = qval;
	  }
	}
	return a_max;
      }
    };

    /**
     * @short KTDQ algorithm
     *
     * ACTION_ITERATOR Used for the argmax function.
     */
    template<typename SA_ARCHITECTURE,
	     typename SA_GREEDY,
	     typename ACTION_ITERATOR,
	     typename KTD_PARAM>
    class KTDQ : public KTD<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM> {
      
    private:
      
      const SA_GREEDY& greedy;
      
    public: 

      typedef KTD<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM>            super_type;
      typedef KTDQ<SA_ARCHITECTURE,SA_GREEDY,ACTION_ITERATOR,KTD_PARAM> self_type;
      typedef typename super_type::state_type                           state_type;
      typedef typename super_type::action_type                          action_type;
      typedef typename super_type::input_type                           input_type;
      typedef typename super_type::output_type                          output_type;

      KTDQ(const SA_ARCHITECTURE& a,
	   const SA_GREEDY& g,
	   const ACTION_ITERATOR& it,
	   const KTD_PARAM& p) : super_type(a,it,p), greedy(g) {}
      KTDQ(const self_type& cp) : super_type(cp), greedy(cp.greedy) {}

      self_type& operator=(const self_type& cp) {
	if(this != &cp) {
	  this->super_type::operator=(cp);
	  greedy = cp.greedy;
	}
	return *this;
      }

      virtual ~KTDQ(void) {}

      virtual action_type selectAction(const input_type& next,
				       output_type& vValue,
				       unsigned int i) const {
	const state_type&  nextState = next.s;
	gsl_vector sigmaPoint = gsl_matrix_column(this->sigmaPointsSet,i).vector;
	
	return greedy(this->archi,
		      &sigmaPoint,
		      nextState,
		      vValue);
      }
    };

    template<typename SA_ARCHITECTURE,
	     typename SA_GREEDY,
	     typename ACTION_ITERATOR,
	     typename KTD_PARAM>
    KTDQ<SA_ARCHITECTURE,SA_GREEDY,ACTION_ITERATOR,KTD_PARAM> ktd_q(const SA_ARCHITECTURE& a,
								    const SA_GREEDY& g,
								    const ACTION_ITERATOR& it,
								    const KTD_PARAM& p) {
      return KTDQ<SA_ARCHITECTURE,SA_GREEDY,ACTION_ITERATOR,KTD_PARAM>(a,g,it,p); 
    }



    /**
     * @short KTDSARSA algorithm
     *
     * ACTION_ITERATOR Used for the argmax function.
     */
    template<typename SA_ARCHITECTURE,
	     typename ACTION_ITERATOR,
	     typename KTD_PARAM>
    class KTDSARSA : public KTD<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM> {
      
    public: 
      typedef KTDSARSA<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM> self_type;
      typedef KTD<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM>      super_type;
      typedef typename super_type::state_type                     state_type;
      typedef typename super_type::action_type                    action_type;
      typedef typename super_type::output_type                    output_type;
      typedef typename super_type::input_type                     input_type;

      KTDSARSA(const SA_ARCHITECTURE& a,
	       const ACTION_ITERATOR& it,
	       const KTD_PARAM& p) : super_type(a,it,p) {}
      KTDSARSA(const self_type& cp) : super_type(cp) {}

      self_type& operator=(const self_type& cp) {
	  return this->super_type::operator=(cp);
      }
      virtual ~KTDSARSA(void) {}

      virtual action_type selectAction(const input_type& next,
				       output_type& vValue,
				       unsigned int i) const {
	action_type a = next.a;
	gsl_vector sigmaPoint = gsl_matrix_column(this->sigmaPointsSet,i).vector;
	vValue = this->archi(&sigmaPoint,
			     next.s,a);
	return a;
      }
    };

    template<typename SA_ARCHITECTURE,
	     typename ACTION_ITERATOR,
	     typename KTD_PARAM>
    KTDSARSA<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM> ktd_sarsa(const SA_ARCHITECTURE& a,
								  const ACTION_ITERATOR& it,
								  const KTD_PARAM& p) {
      return KTDSARSA<SA_ARCHITECTURE,ACTION_ITERATOR,KTD_PARAM>(a,it,p);
    }
    
  }
}
  
#endif
