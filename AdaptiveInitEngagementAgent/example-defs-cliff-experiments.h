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
// common definitions.

#define NB_EPISODES            10000
#define MAX_EPISODE_DURATION     100
#define FRAME_PERIOD              25
#define MIN_V                    -50

#include <iomanip>

template<typename CRITIC>
void make_experiment(const Param& param,
		     CRITIC&      critic) {
  Simulator                simulator;
  ActionToss               a_toss;
  int                      episode,frame;
  Transition               transition;
  Reward                   max_q;
  int                      episode_length;
   
  auto explore_agent = rl::agent::epsilon_greedy(critic,a_toss,param); 
  auto test_agent    = rl::agent::greedy(critic);   


  rl::problem::cliff_walking::CliffPixel drawing_data[Cliff::size]; 
  
  
  std::cout << std::endl << std::endl;
  for(episode = 0, frame = 0;
      episode < NB_EPISODES;
      ++episode) {

    std::cout << "running episode " << std::setw(6) << episode+1
	      << "/" << NB_EPISODES
	      << "    \r" << std::flush;
    simulator.restart();
    rl::episode::sa::run_and_learn(simulator,explore_agent,critic,transition,0,episode_length);

    if(episode % FRAME_PERIOD == 0) {
      for(S s=Cliff::start; s <= Cliff::goal; ++s) {
	critic.argmax(s,max_q);
	drawing_data[s].value = max_q;
	drawing_data[s].visited = false;
      }
      
      // Let us be greedy on the policy we have found, and draw the path.
      try {
      	simulator.restart();
	for(int k=0;k<MAX_EPISODE_DURATION;++k) {
      	  drawing_data[simulator.sense()].visited = true;
      	  simulator.timeStep(test_agent.policy(simulator.sense()));
      	} 
      }
      catch(rl::exception::Terminal e) {
      }
      
      Cliff::draw("rllib",frame++,drawing_data,MIN_V,0);
    }
  }
  std::cout << std::endl
	    << std::endl;


  std::string command;
  
  command = "find . -name \"rllib-*.ppm\" -exec convert \\{} -filter Box -resize 192x64 -quality 100 \\{}.jpg \\;";
  std::cout << "Executing : " << command << std::endl;
  system(command.c_str());

  command = "ffmpeg -i rllib-%06d.ppm.jpg -r 5 rllib.avi";
  std::cout << "Executing : " << command << std::endl;
  system(command.c_str());

  command = "find . -name \"rllib-*.ppm\" -exec rm \\{} \\;";
  std::cout << "Executing : " << command << std::endl;
  system(command.c_str());

  command = "find . -name \"rllib-*.ppm.jpg\" -exec rm \\{} \\;";
  std::cout << "Executing : " << command << std::endl;
  system(command.c_str());
}
