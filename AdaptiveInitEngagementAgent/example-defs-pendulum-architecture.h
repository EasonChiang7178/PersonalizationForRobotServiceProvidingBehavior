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

// We need an action iterator to toss random actions but also to
// iterate on all the possible action values.
typedef rl::Iterator<A,
		     rl::problem::inverted_pendulum::actionNone,  
		     rl::problem::inverted_pendulum::actionRight> ActionIterator;


// This feature justs transform (s,a) into a vector 
// (angle,speed,action_is_None,action_is_Left,action_is_Right).
class DirectFeature {
public:

  typedef SA input_type;
  typedef S  state_type;
  typedef A  action_type;

  int dimension(void) const {return 5;}

  void operator()(const input_type& x, 
		  gsl_vector *phi) const {
    (*this)(x.s,x.a,phi);
  }

  void operator()(const state_type&  s, 
		  const action_type& a, 
		  gsl_vector *phi) const {
    gsl_vector_set_zero(phi);
    gsl_vector_set(phi,0,s.angle);
    gsl_vector_set(phi,1,s.speed);
    switch(a) {
    case rl::problem::inverted_pendulum::actionNone:
      gsl_vector_set(phi,2,1.0);
      break;
    case rl::problem::inverted_pendulum::actionLeft:
      gsl_vector_set(phi,3,1.0);
      break;
    case rl::problem::inverted_pendulum::actionRight:
      gsl_vector_set(phi,4,1.0);
      break;
    default:
      throw rl::problem::inverted_pendulum::BadAction(" in Feature::operator()");
    }
  }
};

// This is the feature based on gaussian RBF.
class RBFFeature {
private:
  
  double angle[3];
  double speed[3];

public:

  typedef SA input_type;
  typedef S  state_type;
  typedef A  action_type;

  RBFFeature(void) {
    // These are the centers of the gaussian that spans S.
    angle[0] = -M_PI_4;
    angle[1] =       0;
    angle[2] = +M_PI_4;
    speed[0] =      -1;
    speed[1] =       0;
    speed[2] =      +1;
  }

  int dimension(void) const {return 30;}

  void operator()(const input_type& x, 
		  gsl_vector *phi) const {
    (*this)(x.s,x.a,phi);
  }

  void operator()(const state_type&  s, 
		  const action_type& a, 
		  gsl_vector *phi) const {
    int action_offset;
    int i,j,k;
    double dangle,dspeed;

    if(phi == (gsl_vector*)0)
      throw rl::exception::NullVectorPtr("in Feature::operator()");
    else if((int)(phi->size) != dimension())
      throw rl::exception::BadVectorSize(phi->size,dimension(),"in Feature::operator()");

    switch(a) {
    case rl::problem::inverted_pendulum::actionNone:
      action_offset=0;
      break;
    case rl::problem::inverted_pendulum::actionLeft:
      action_offset=10;
      break;
    case rl::problem::inverted_pendulum::actionRight:
      action_offset=20;
      break;
    default:
      throw rl::problem::inverted_pendulum::BadAction("in Feature::operator()");
    }

    gsl_vector_set_zero(phi);
    for(i=0,k=action_offset+1;i<3;++i) {
      dangle  = s.angle - angle[i];
      dangle *= dangle;
      for(j=0;j<3;++j,++k) {
	dspeed  = s.speed - speed[j];
	dspeed *= dspeed;
	gsl_vector_set(phi,k,exp(-.5*(dangle+dspeed))); 
      }
      gsl_vector_set(phi,action_offset,1);
    }
  }
};

