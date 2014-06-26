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

#ifndef rlRANDOM_H
#define rlRANDOM_H

#include <cstdlib>

namespace rl {
  /**
   * @short  This encapsulate random number generator.
   * @author <a href="mailto:Herve.Frezza-Buet@supelec.fr">Herve.Frezza-Buet@supelec.fr</a>
   */
  class random {
  public:

    /**
     * Initialization of random seed.
     */
    static void seed(unsigned int s) {srand(s);}
    
    /**
     * @return A random value in [0..1[
     */
    static double toss(void) {return rand()/(1.0+RAND_MAX);}

    /**
     * @return A random value in [min..max[
     */
    static double toss(double min,double max) {return min + (max-min)*toss();}
  };
}

#endif
