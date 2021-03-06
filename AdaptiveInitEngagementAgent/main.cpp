
/* Standrad Included Library */
#include <iostream>
#include <cstdio>
using namespace std;

/* Third-party Library */
	// To use rllib
#include <rl.h>

/** Declration of Variables **/
typedef rl::problem::adaptive_interruption::AdaptiveInterruption AdaptiveInterruption;
typedef rl::problem::adaptive_interruption::Simulator<AdaptiveInterruption> Simulator;

class Param {
public: 
  double gamma(void)   const {return .99;}
  double alpha(void)   const {return .05;}
  double epsilon(void) const {return 0.2;}
};

// Definition of Reward, S, A, SA, Transition and TransitionSet.
#include "example-defs-transition.h"

// Definition of ActionIterator, TabularQ
#include "AdaptiveInterruption_TabularQ.h"

//** Problem Dependent Variable Setting **//
#define NB_EPISODES				 10
#define MAX_EPISODE_DURATION     100

//=============================================================================
int main(int argc, char* argv[]) {
	string userName = "S";

	TabularQ   q;
	Param      param;
	ActionIterator a_iter;

	auto greedy        = rl::sa::greedy(a_iter); 
	auto argmax_critic = rl::sa::q_learning(q,greedy,param);

	Simulator				simulator;
	ActionToss				a_toss;
	int						episode,frame;
	TransitionSet			transition;
	Reward					sum = 0.0;
	int						episode_length;

	auto explore_agent	= rl::agent::epsilon_greedy(argmax_critic, a_toss, param);
	auto test_agent		= rl::agent::greedy(argmax_critic);

	cout << "\n\t\t< Human-aware Decision Maker >" << endl;
	cout << "> Please press ENTER to start..." << endl;
	getchar();

	/* Load the parnter data and set the paramter to Q-learning agent */
	gsl_vector * dataLearned = gsl_vector_alloc (90);
	FILE * fUserData = fopen ((userName + ".dat").c_str(), "r");
	if (fUserData != NULL) {
		cout << "\n> INFO: Loading user, " << userName << ", preference data...\n" << endl;
		gsl_vector_fscanf (fUserData, dataLearned);
		fclose (fUserData);
		argmax_critic = dataLearned;
	}
	gsl_vector_free(dataLearned);

	for(episode = 0, frame = 0; episode < NB_EPISODES; ++episode) {
		std::cout << ">\t\t< Episode " << std::setw(6) << episode + 1 << "/" << NB_EPISODES << " >" << endl;
		
		simulator.restart();
		rl::episode::sa::run_and_learn(simulator, explore_agent, argmax_critic, transition, 0, episode_length);
		
		for (auto iter = transition.begin(); iter != transition.end(); ++iter) {
			sum += (*iter).reward(); // *iter is a transition
			//cout << "> Action: " << (*iter).currentAction() << ", State_toa: " << (*iter).currentState().robotBelief << ", State_alr: " << (*iter).currentState().personAttention << endl;
		}
		cout << "> Total reward during episode : " << sum << std::endl;
	
		/* Store the param into file */
		const gsl_vector* parnterPreference = argmax_critic.parameter();

		FILE * fparam = fopen((userName + ".dat").c_str(), "w");
		gsl_vector_fprintf (fparam, parnterPreference, "%.5g");
		fclose (fparam);

		/* Store the reward into file */
		FILE* freward = fopen((userName + "Reward.txt").c_str(), "a+");
		fprintf(freward, "%f\n", sum);
		fclose(freward);

		/* Clear data for next episode */
		sum = 0.0;
		transition.clear();
		transition.resize(0);

		//getchar();
	}

	//try {
	//	simulator.restart();
	//	for (int k = 0; k < MAX_EPISODE_DURATION; ++k)
	//		simulator.timeStep(test_agent.policy(simulator.sense()));
	//} catch(rl::exception::Terminal e) 
	//{ }

	transition.clear();
	transition.resize(0);

	simulator.restart();
	rl::episode::sa::run_and_collect(simulator, test_agent, transition, 0, episode_length);

		// Let us read the collected transitions
	for (auto iter = transition.begin(), iter_end = transition.end(); iter != transition.end(); ++iter) {
		sum += (*iter).reward(); // *iter is a transition.
		//cout << "> Action: " << (*iter).currentAction() << ", State_toa: " << (*iter).currentState().robotBelief << ", State_alr: " << (*iter).currentState().personAttention << endl;
	}
	cout << "Total reward during episode : " << sum << std::endl;

	cout << endl;
	cout << "Please press ENTER to left..." << endl;
	getchar();

	return 0;
}
