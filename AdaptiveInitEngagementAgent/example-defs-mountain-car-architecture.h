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

// This file is a piece of code included by our examples. It gathers
// common definition of types.



typedef rl::Iterator<A,
		     rl::problem::mountain_car::actionNone,
		     rl::problem::mountain_car::actionForward> ActionIterator;

// This feature just transforms (s,a) into a vector 
// (position,speed,action_is_None,action_is_Backward,action_is_Forward).
class DirectFeature {
public:

  typedef SA  input_type;
  typedef S   state_type;
  typedef A   action_type;

  int dimension(void) const {return 5;}

  void operator()(const input_type& x, 
		  gsl_vector *phi) const {
    (*this)(x.s,x.a,phi);
  }

  void operator()(const state_type&  s, 
		  const action_type& a, 
		  gsl_vector *phi) const {
    gsl_vector_set_zero(phi);

    // We normalize position and speed
    gsl_vector_set(phi,0,
		   (s.position - Simulator::param_type::minPosition())
		   /(Simulator::param_type::maxPosition()- Simulator::param_type::minPosition()));
    gsl_vector_set(phi,1,
		   (s.speed - Simulator::param_type::minSpeed())
		   /(Simulator::param_type::maxSpeed()- Simulator::param_type::minSpeed()));
    switch(a) {
    case rl::problem::mountain_car::actionNone:
      gsl_vector_set(phi,2,1.0);
      break;
    case rl::problem::mountain_car::actionBackward:
      gsl_vector_set(phi,3,1.0);
      break;
    case rl::problem::mountain_car::actionForward:
      gsl_vector_set(phi,4,1.0);
      break;
    default:
      throw rl::problem::mountain_car::BadAction(" in Feature::operator()");
    }
  }
};

// This is the feature based on gaussian RBF.
#define SPLIT 5
#define SIGMA (1/((SPLIT)-1.0))
#define SIGMA2 ((SIGMA)*(SIGMA))

#define DELTA_POSITION (Simulator::param_type::maxPosition()- Simulator::param_type::minPosition())
#define DELTA_POSITION2 ((DELTA_POSITION)*(DELTA_POSITION))

#define DELTA_SPEED (Simulator::param_type::maxSpeed()- Simulator::param_type::minSpeed())
#define DELTA_SPEED2 ((DELTA_SPEED)*(DELTA_SPEED))


class RBFFeature {
private:
  
  double position[SPLIT];
  double speed[SPLIT];

public:
  typedef SA                   sa_type;
  typedef sa_type              input_type;
  typedef sa_type::state_type  state_type;
  typedef sa_type::action_type action_type;

  RBFFeature(void) {
    // These are the centers of the gaussian that spans S.
    int i;
    
    double position_step = DELTA_POSITION/(SPLIT-1.0);
    double speed_step    = DELTA_SPEED/(SPLIT-1.0);
    
    for(i=0;i<SPLIT;++i)
      position[i] = i*position_step 
	+ Simulator::param_type::minPosition();
    
    for(i=0;i<SPLIT;++i)
      speed[i] = i*speed_step 
	+ Simulator::param_type::minSpeed();
  }
  
  int dimension(void) const {return (SPLIT*SPLIT+1)*3;};

  void operator()(const input_type& x, 
		  gsl_vector *phi) const {
    (*this)(x.s,x.a,phi);
  }

  void operator()(const state_type&  s, 
		  const action_type& a, 
		  gsl_vector *phi) const {
    int action_offset;
    int i,j,k;
    double dposition,dspeed;

    if(phi == (gsl_vector*)0)
      throw rl::exception::NullVectorPtr("in Feature::operator()");
    else if((int)(phi->size) != dimension())
      throw rl::exception::BadVectorSize(phi->size,dimension(),"in Feature::operator()");

    switch(a) {
    case rl::problem::mountain_car::actionNone:
      action_offset=0;
      break;
    case rl::problem::mountain_car::actionBackward:
      action_offset=(SPLIT*SPLIT+1);
      break;
    case rl::problem::mountain_car::actionForward:
      action_offset=2*(SPLIT*SPLIT+1);
      break;
    default:
      throw rl::problem::inverted_pendulum::BadAction("in Feature::operator()");
    }

    gsl_vector_set_zero(phi);
    for(i=0,k=action_offset+1;i<SPLIT;++i) {
      dposition  = s.position - position[i];
      dposition *= dposition;
      dposition /= 2*SIGMA2*DELTA_POSITION2;
      for(j=0;j<SPLIT;++j,++k) {
	dspeed  = s.speed - speed[j];
	dspeed *= dspeed;
	dspeed /= 2*SIGMA2*DELTA_SPEED2;
	gsl_vector_set(phi,k,exp(-dposition-dspeed)); 
      }
      gsl_vector_set(phi,action_offset,1);
    }
  }
 
};

