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

#ifndef rlTYPES_H
#define rlTYPES_H

#include <rlRandom.h>
#include <rlException.h>
#include <rlConcept.h>
#include <iostream>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

std::ostream& operator<<(std::ostream& os, const gsl_vector* v);
/**
 * @param v Is a pointer that will be freed and reallocated by the call. <b>avoid uninitialized v !</b> (use 0 at least)
 */
std::istream& operator>>(std::istream& is, gsl_vector*& v);
std::ostream& operator<<(std::ostream& os, const gsl_matrix* m);
/**
 * @param m Is a pointer that will be freed and reallocated by the call. <b>avoid uninitialized m !</b> (use 0 at least)
 */
std::istream& operator>>(std::istream& is, gsl_matrix*& m);

namespace rl {
  class Serializer;
}
std::ostream& operator<<(std::ostream& os, const rl::Serializer& s);
std::istream& operator>>(std::istream& is, rl::Serializer& s);

namespace rl {

  /**
   * @short Helps doxygen to document that a class fits a concept.
   */
  template<typename CONCEPT>
  class fits {};

  /**
   * @short Helps doxygen to document that some nested types should be equal.
   */
  template<typename CONCEPT1,typename CONCEPT2>
  class have_to_match {};

  /**
   * @short State action pair.
   */
  template<typename STATE, typename ACTION>
  class SA {
  public:

    typedef STATE   state_type;
    typedef ACTION action_type;

    state_type  s;
    action_type a;

    SA(void) : s(), a() {}
    SA(const SA<state_type,action_type>& sa) : s(sa.s), a(sa.a) {}
    SA<state_type,action_type>& operator=(const SA<state_type,action_type>& sa) {
      if(this != &sa) {
	s = sa.s;
	a = sa.a;
      }
      return *this;
    }
    ~SA(void) {}
    
    SA(const state_type&  state,
       const action_type& action) : s(state), a(action) {}
  };

  template<typename STATE, typename ACTION>
  SA<STATE,ACTION> sa_pair(const STATE& s, const ACTION& a) {
    return SA<STATE,ACTION>(s,a);
  }
  

  
  /**
   * @short This allows to iterate on an discrete set.
   */
  template<typename ANY,int FIRST,int LAST>
  class Iterator 
    : public rl::fits<rl::concept::SizedIterator<ANY> >,
      public rl::fits<rl::concept::Toss<ANY> >{
  public:
    
    typedef ANY value_type;

    int size(void) const {return LAST-FIRST+1;}
        
    /**
     * @param rank in [0,size[
     */
    ANY value(int rank) const {
      if(rank < 0 || rank >= size())
	throw rl::exception::BadIteratorRank(size(),rank," in rl::Iterator::value()");
      return (ANY)(FIRST+rank);
    }

    ANY first(void) const {
      return (ANY)FIRST;
    }
    
    bool hasNext(const ANY& current) const {
      return (int)current < LAST;
    }
    
    ANY next(const ANY& current) const {
      return (ANY)((int)current+1);
    }

    ANY random(void) const {
      return (ANY)int(rl::random::toss(FIRST,LAST+1));
    }
  };

  /**
   * @short upper class for using << and >>
   *
   * Inherit from this class, override virtual read and write, and <<
   * >> will work.
   */
  class Serializer {
  protected:
    virtual void read(std::istream& is)=0;
    virtual void write(std::ostream& os) const=0;
    friend std::ostream& ::operator<<(std::ostream& os, const rl::Serializer& v);
    friend std::istream& ::operator>>(std::istream& is, rl::Serializer& v);
  };

}


// GSL serialization



inline std::ostream& operator<<(std::ostream& os, const gsl_vector* v) {
  os << "[ " << v->size
     << " :";
  for(unsigned int i=0;i<v->size;++i)
    os << " " << gsl_vector_get(v,i);
  os << "]";
    
  return os;
}

inline std::istream& operator>>(std::istream& is, gsl_vector*& v) {
  char c;
  double value;
  unsigned int i,size;
  is >> c >> size >> c;
  gsl_vector_free(v);
  v = gsl_vector_alloc(size);
  for(i=0;i<size;++i) {
    is >> value;
    gsl_vector_set(v,i,value);
  }
  is >> c ;
  return is;
}

inline std::ostream& operator<<(std::ostream& os, const gsl_matrix* m) {
  unsigned int i,j;
  os << "[ " << m->size1 << 'x' <<  m->size2
     << " :";
  for(i=0;i<m->size1;++i)
    for(j=0;j<m->size2;++j)
      os << " " << gsl_matrix_get(m,i,j);
  os << "]";
  return os;
}

inline std::istream& operator>>(std::istream& is, gsl_matrix*& m) {
  char c;
  double value;
  unsigned int i,j,size1,size2;
  is >> c >> size1 >> c >> size2 >> c;
  gsl_matrix_free(m);
  m = gsl_matrix_alloc(size1,size2);
  for(i=0;i<size1;++i) 
    for(j=0;j<size2;++j) {
      is >> value;
      gsl_matrix_set(m,i,j,value);
  }
  is >> c ;
  return is;
}


inline std::ostream& operator<<(std::ostream& os, const rl::Serializer& s) {
  s.write(os);
  return os;
}

inline std::istream& operator>>(std::istream& is, rl::Serializer& s) {
  s.read(is);
  return is;
}





#endif
