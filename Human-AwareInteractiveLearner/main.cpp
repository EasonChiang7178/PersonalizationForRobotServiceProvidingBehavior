
/* Standrad Included Library */
#include <iostream>
#include <cstdio>
using namespace std;

/* Third-party Library */
	// To use rllib
#include <rl.h>

#include "rl-InteractiveLearner.h"

/** Declration of Variables **/
typedef rl::problem::adaptive_interruption::AdaptiveInterruption AdaptiveInterruption;
typedef rl::problem::adaptive_interruption::Simulator<AdaptiveInterruption> Simulator;

//class Param {
//public: 
//  double gamma(void)   const {return .99;}
//  double alpha(void)   const {return .05;}
//  double epsilon(void) const {return 0.2;}
//};

class Param {
public:
	double temperature(void) const {return 32.0;}
	double gamma(void)   const {return 1;}
	double alpha(void)   const {return .75;}
};

// Definition of Reward, S, A, SA, Transition and TransitionSet.
#include "example-defs-transition.h"

// Definition of ActionIterator, TabularQ
#include "InteractiveLearner_TabularQ.h"

//** Problem Dependent Variable Setting **//
#define NB_EPISODES				35
//#define MAX_EPISODE_DURATION	100
#define PARTNERNAME				"C1_Shin"
//#define TESTRESULT

//=============================================================================
int main(int argc, char* argv[]) {
	string userName(PARTNERNAME);

	TabularQ		q;
	Param			param;
	ActionIterator	a_iter;

	auto greedy			= rl::sa::greedy(a_iter);
	auto argmax_critic	= rl::sa::q_learning(q, greedy, param);

	Simulator				simulator;
	//ActionToss				a_toss;
	int						episode,frame;
	TransitionSet			transition;
	Reward					sum = 0.0;
	int						episode_length;

	//auto explore_agent	= rl::agent::epsilon_greedy(argmax_critic, a_toss, param);
	//auto explore_agent	= rl::agent::softmax(argmax_critic, a_iter, param);
	auto explore_agent	= rl::agent::diberateactionselection(argmax_critic, a_iter, param);
	auto test_agent		= rl::agent::greedy(argmax_critic);

	cout << "\n\t\t< Human-aware Interactive Learner >" << endl;
	cout << "> Please press ENTER to start..." << endl;
	getchar();

	/* Load the parnter data and set the paramter to Q-learning agent */
	gsl_vector * dataLearned = gsl_vector_alloc (AdaptiveInterruption::size * rl::problem::adaptive_interruption::actionSize);
	FILE * fUserData = fopen ((userName + ".dat").c_str(), "r");
	if (fUserData != NULL) {
		cout << "\n> INFO: Loading user, " << userName << ", preference data...\n" << endl;
		gsl_vector_fscanf (fUserData, dataLearned);
		fclose (fUserData);
		argmax_critic = dataLearned;
	}
	gsl_vector_free(dataLearned);

	/* Interactive Learning */
#ifndef TESTRESULT
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
#endif

#ifdef TESTRESULT
	/* Test Result! */
	simulator.restart();
	rl::episode::sa::run_and_collect(simulator, test_agent, transition, 0, episode_length);

		// Let us read the collected transitions
	for (auto iter = transition.begin(), iter_end = transition.end(); iter != transition.end(); ++iter) {
		sum += (*iter).reward(); // *iter is a transition.
		//cout << "> Action: " << (*iter).currentAction() << ", State_toa: " << (*iter).currentState().robotBelief << ", State_alr: " << (*iter).currentState().personAttention << endl;
	}
	cout << "Total reward during episode : " << sum << std::endl;
#endif //TESTRESULT

	cout << endl;
	cout << "Please press ENTER to left..." << endl;
	getchar();

	return 0;
}
