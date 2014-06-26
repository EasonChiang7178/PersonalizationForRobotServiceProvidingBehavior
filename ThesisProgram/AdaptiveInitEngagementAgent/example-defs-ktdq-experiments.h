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


template<typename ARGMAX_CRITIC>
void make_experiment(ARGMAX_CRITIC& critic) {
  int                      episode,step,episode_length;
  std::ofstream            ofile;
  std::ifstream            ifile;

  Simulator                simulator;
  Transition               transition;
  ActionIterator           a_toss;
  ARGMAX_CRITIC            critic_loaded = critic;

  auto explore_agent       = rl::agent::random(S(),a_toss);
  auto greedy_agent        = rl::agent::greedy(critic);
  auto greedy_agent_loaded = rl::agent::greedy(critic_loaded);

  try {
    step = 0;

    // Let us initialize the random seed.
    rl::random::seed(getpid());

    for(episode = 0; episode < NB_OF_EPISODES; ++episode) {
      simulator.setPhase(Simulator::phase_type()); 
      rl::episode::sa::run_and_learn(simulator,explore_agent,critic,transition,
				     MAX_EPISODE_LENGTH,episode_length);

      if((episode % TEST_PERIOD)==0) {
	++step;
	test_iteration(greedy_agent,step);
      }
    }

    // Now, we can save the ktdq object.
    std::cout << "Writing ktdq.data" << std::endl;
    ofile.open("ktdq.data");
    if(!ofile)
      std::cerr << "cannot open file for writing" << std::endl;
    else {
      ofile << critic;
      ofile.close();
    }

    // You can load back with <<
    std::cout << "Reading ktdq.data" << std::endl;
    ifile.open("ktdq.data");
    if(!ifile)
      std::cerr << "cannot open file for reading" << std::endl;
    else {
      ifile >> critic_loaded;
      ifile.close();
    }

    // let us try this loaded ktdq
    test_iteration(greedy_agent_loaded,step);   
  }
  catch(rl::exception::Any& e) {
    std::cerr << "Exception caught : " << e.what() << std::endl;
  }
}
