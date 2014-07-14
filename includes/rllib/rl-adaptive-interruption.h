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

#ifndef rlADAPTIVE_INTERRUPTION_H
#define rlADAPTIVE_INTERRUPTION_H

//#define SIMULATION
#define PARTNERNAME "TEST"

/* Standrad Included Library */
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>

#ifdef SIMULATION
#include <random>
#endif

using namespace std;

/* Third-party Library */
#include <rlException.h>
#include <gsl/gsl_vector.h>
#include "../RobotAction/RobotAction.h"

namespace rl {
	namespace problem {
		namespace adaptive_interruption {

#ifdef SIMULATION
			default_random_engine generator;
#endif

			/* Define the action space for the robot */
			typedef enum Action {HeadShake,
								 Rotation,
								 Approach,
								 CallName,
								 ArmWave,
								 MoveToAroundOfPerson,
								 MakeSound} RobotActionSet;
			enum {actionSize = 7};

			/* Some exceptions for state and action consistancy */
				// Can be throwed in inapprioate approach (When the robot is already too closed to the person
			class BadAction : public rl::exception::Any {
				public:
					BadAction(std::string comment) : Any(std::string("Bad action performed : ") + comment) {}
			};
				// Where could be used?
			class BadState : public rl::exception::Any {
				public:
					BadState(std::string comment) : Any(std::string("Bad state found : ") + comment) {}
			};

			//** Define the phase space for the robot, Fitting rl::concept::Phase **//
				// Here start describing our phase state space
			class AdaptiveInterruption {
				public:
					typedef enum ALR {Neglect,
									  Medium,
									  High} ALR;
					enum {ALRSize = 3};

					typedef enum ToA {None,
									  HasPersonNoticedMe,
									  DoesPersonLookingMe,
									  LoseAttention,
									  Engaged} ToA;
					enum {ToASize = 5};

						// Pair of states
					ALR personAttention;
					ToA robotBelief;

					enum {
						size = ALRSize * ToASize
					};

					/** Fitting necessary constructors **/
						// Constructor
					AdaptiveInterruption() {
						personAttention = Neglect;
						robotBelief = None;
					}
						// Copy constructor
					AdaptiveInterruption(const AdaptiveInterruption& p) {
						personAttention = p.personAttention;
						robotBelief = p.robotBelief;
					}
						// Overloading assignment operator
					AdaptiveInterruption& operator=(const AdaptiveInterruption& p) {
						personAttention = p.personAttention;
						robotBelief = p.robotBelief;

						return *this;
					}
			};

			//** Fitting the Simulator, rl::concept **//
			template<typename ADAPTIVE_INTERRUPTION>
			class Simulator {
				/** Fitting the rl::concept::Simulator */
				public:
					typedef typename ADAPTIVE_INTERRUPTION		phase_type;
					typedef phase_type							observation_type;
					typedef RobotActionSet						action_type;
					typedef double								reward_type;

				private:
						// For representing state (phase)
					phase_type current_state;
						// For representing reward
					double r;
						// For overall interation time bound
					int timeOutBound, noticedTimeOutBound;
						// The seconds for Stable ATR
					int stableATRTimeBound;
						// To store the interaction time
					time_t interaction_start, interaction_current, ATR_high_start, ATR_high_current;
					/* Define reward situation */
					static inline double EngagedReward()		{return 100;}
					static inline double HaveNoticedReward()	{return 20;}
					static inline double HumanActiveReward()	{return 20;}
					static inline double LoseAttentionReward()	{return -100;}
					static inline double RejectReward()			{return -50;}
					static inline double StepReward()			{return -1;}
						// Perform the robot action
					RobotAction robot;
						// Test variable for personAttention
					int userInput;
						// For approach using, where the robot is
					bool goFront, approached;

					AdaptiveInterruption::ALR preAttentionState;

				public:
					/* User defined functions */
					void restart(void) {
						setPhase(phase_type());
						interaction_start = time(NULL);
						approached = false;
						goFront = false;
#ifndef SIMULATION
						robot.toPoint(0.0, 0.0, 0.0);
						//robot.turningFace(-45.0);
						robot.turnFaceToHuman();
						Sleep(5000);
#endif
					}

					void setPhase(const phase_type& s) {
						current_state = s;
							// Check the validation of the input state
						if (s.personAttention < 0 || s.personAttention > ADAPTIVE_INTERRUPTION::ALRSize ||
							s.robotBelief < 0 || s.robotBelief > ADAPTIVE_INTERRUPTION::ToASize) {
							std::ostringstream ostr;

							//ostr << "Simulator::setPhase(" << s << ")";
							throw BadState(ostr.str());
						}
					}

					/** Fitting sense function **/
					const observation_type& sense(void) const {
						return current_state;
					}
					
					/** Fitting reward function **/
					reward_type reward(void) const {
						return r;
					}

					/** Fitting timeStep function **/
					void timeStep(const action_type& a) {
						switch(current_state.robotBelief) {
							case ADAPTIVE_INTERRUPTION::LoseAttention:
							case ADAPTIVE_INTERRUPTION::Engaged:
								stepGoal();
								break;

							case ADAPTIVE_INTERRUPTION::DoesPersonLookingMe:
								TryToinitiatieInteraction();
								break;

							default:
								step(a);
								break;
						}
#ifndef SIMULATION
						/* Query the next human attention level */
						//robot.sensingATR(200);
						//preAttentionState = static_cast< ADAPTIVE_INTERRUPTION::ALR >(robot.getATR());
						Sleep(5000);
						robot.sensingATR(200);
						current_state.personAttention = static_cast< ADAPTIVE_INTERRUPTION::ALR >(robot.getATR());
#else
						/* Random generate HAE for simulation */
						//std::normal_distribution< double > distribution(static_cast< double >(ADAPTIVE_INTERRUPTION::ALRSize - 2), 2.0);
						//int randomNum = static_cast< int >(distribution(generator));
						//if (randomNum > ADAPTIVE_INTERRUPTION::ALRSize - 1) randomNum = ADAPTIVE_INTERRUPTION::ALRSize - 1;
						//if (randomNum < 0) randomNum = 0;
						//current_state.personAttention = static_cast< ADAPTIVE_INTERRUPTION::ALR > (randomNum);

						//current_state.personAttention = (ADAPTIVE_INTERRUPTION::ALR) 2;

						cout << "> INFO: Your attention level to robot: ";
						cin >> userInput;
						preAttentionState = current_state.personAttention;
						current_state.personAttention = static_cast< ADAPTIVE_INTERRUPTION::ALR > (userInput);
						
#endif

						updateToA();
						cout << "> \tCurrent state, HAR:" << current_state.personAttention << ", ToA: " << current_state.robotBelief << endl;
					}

				private:
						// Enter into the absorbing state
					void stepGoal() {
						r = 0;
						cout << ">\t\t\t<TO NEXT EPISODE>" << endl;
						throw rl::exception::Terminal("Transition from absorbing state");
					}
						// Perform RobotAction
					void step(const action_type a) {
						r = StepReward();
						switch(a) {
//							case DoNothing:
//								cout << "> RobotAction: DoNothing" << endl;
//#ifndef SIMULATION
//								robot.doNothing(3000);
//#endif
//								break;

							case ArmWave:
								cout << "> RobotAction: ArmWave" << endl;
#ifndef SIMULATION
								robot.armWave(1);
#endif
								break;

							case HeadShake:
								cout << "> RobotAction: HeadShake" << endl;
#ifndef SIMULATION
								if (approached == false && goFront == false)
									robot.headShake(15, 0);
								else
									robot.headShake(15, 0);
								robot.turnFaceToHuman();
#endif
								break;

							case Rotation:
								cout << "> RobotAction: Rotation" << endl;
#ifndef SIMULATION
							robot.rotation(15);
#endif
								break;

							case MoveToAroundOfPerson:
								cout << "> RobotAction: MoveToFrontOfPerson" << endl;
#ifndef SIMULATION
								//if (approached == false && goFront == false) {
								//	robot.toPoint(1.8, 0.0, 90.0);
								//	goFront = true;
								//} else if (goFront == false) {
								//	robot.toPoint(1.8, 0.6, 90.0);
								//	goFront = true;
								//} else
								//	cout << "> WARNING: INVALID APPROACH" << endl;
								robot.movingToAroundOfHuman(0, 1.5, -30.0);
#endif
								break;

							case Approach:
								cout << "> RobotAction: Approach" << endl;
#ifndef SIMULATION
								//if (goFront == false && approached == false) {
								//	robot.toPoint(0.6, 0.6, 45.0);
								//	approached = true;
								//} else if (approached == false) {
								//	robot.toPoint(1.8, 0.6, 90.0);
								//	approached = true;
								//} else
								//	cout << "> WARNING: INVALID APPROACH" << endl;
								robot.forwardApproach(0, 0.6);
#endif
								break;

							case MakeSound:
								cout << "> RobotAction: MakeSound" << endl;
#ifndef SIMULATION
								robot.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\models\\Soundness\\WindowsMessage.wav");
#endif
								break;

							case CallName:
								cout << "> RobotAction: CallName" << endl;
#ifndef SIMULATION
								robot.speaking(string(PARTNERNAME), 0.7f);
#endif
								break;

							default:
								std::ostringstream ostr;
								ostr << "RobotAction: stepStart(" << a << ")";
								throw BadAction(ostr.str());
						}
					}
	
					void updateToA() {
						switch(current_state.robotBelief) {
							case ADAPTIVE_INTERRUPTION::None:
								/* Check the person is aware of the robot and transition to HasPersonNoticedMe */
								//if (current_state.personAttention == ADAPTIVE_INTERRUPTION::High && preAttentionState == ADAPTIVE_INTERRUPTION::High) {
								if (current_state.personAttention == ADAPTIVE_INTERRUPTION::High) {
									r = HaveNoticedReward();
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::HasPersonNoticedMe;

									ATR_high_start = time(NULL);
								}
								break;

							case ADAPTIVE_INTERRUPTION::HasPersonNoticedMe:


								/* Transition to DoesPersonLookingMe */
								if (current_state.personAttention != ADAPTIVE_INTERRUPTION::High)
									ATR_high_start = time(NULL);
								else
									ATR_high_current = time(NULL);

									// The high attention level last for stableATRTimeBound
								if (difftime(ATR_high_current, ATR_high_start) > stableATRTimeBound)
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::DoesPersonLookingMe;

								interaction_current = time(NULL);
								if (difftime(interaction_current, ATR_high_start) > noticedTimeOutBound) {
									r = LoseAttentionReward();
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
								}
								break;

							case ADAPTIVE_INTERRUPTION::DoesPersonLookingMe:
									// Lose Attetnion
								if (current_state.personAttention == ADAPTIVE_INTERRUPTION::Neglect) {
									r = LoseAttentionReward();
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
								}

								/* Speech feedback from the partner */
									// Engaged if no Rejected or lose attention occurred
								r = EngagedReward();
								current_state.robotBelief = ADAPTIVE_INTERRUPTION::Engaged;
								break;
						}

						/* Check whether the partner reject or accept the interaction */
						robot.setKeywordListened();
						if (robot.getListenContent() == "Rejected") {
							r = RejectReward();
							current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
							robot.resetKeyword("");
							return;
						}
							// The partner initiate the interaction
						if (robot.getListenContent() == "ARIO") {
							r = HumanActiveReward();
							current_state.robotBelief = ADAPTIVE_INTERRUPTION::Engaged;
							robot.resetKeyword("");
							TryToinitiatieInteraction();

							return;
						}

						/* Check the timeout and transition to LoseAttention */
						interaction_current = time(NULL);
						if (difftime(interaction_current, interaction_start) > timeOutBound) {
							r = LoseAttentionReward();
							current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;

							return;
						}
					}

						// Ask the partner whethler he/she can be interrupted
					void TryToinitiatieInteraction() {
#ifndef SIMULATION
						robot.speaking("There is a message left for you, do you want to reply it now?", 0.7f);
						Sleep(3000);
#else
						cout << "> Robot: There is a message left for you, do you want to reply it now?" << endl;
#endif
					}

				public:
					/** Fitting the constructors, rl::concept::Simulator **/
					Simulator(void) : current_state(), robot("192.168.11.4")
					{
#ifndef SIMULATION
						timeOutBound = 180;
						noticedTimeOutBound = 40;
						stableATRTimeBound = 3;
#else
						timeOutBound = 40;
						noticedTimeOutBound = 10;
						stableATRTimeBound = 3;
#endif
						//noticedFrame = 0;
						goFront = false;
						approached = false;
						interaction_start = time(NULL);
					}
					
					Simulator(const Simulator& copy) : current_state(copy.current_state), robot("192.168.11.4")
					{
						timeOutBound = copy.timeOutBound;
						noticedTimeOutBound = copy.noticedTimeOutBound;
						stableATRTimeBound = copy.stableATRTimeBound;
						noticedFrame = 0;
						goFront = copy.goFront;
						approached = copy.approached;
						interaction_start = copy.interaction_start;
					}

					~Simulator(void) {}

					Simulator& operator=(const Simulator& copy) {
						if(this != &copy) {
							timeOutBound = copy.timeOutBound;
							noticedTimeOutBound = copy.noticedTimeOutBound;
							stableATRTimeBound = copy.stableATRTimeBound;
							noticedFrame = 0;
							goFront = copy.goFront;
							approached = copy.approached;
							interaction_start = copy.interaction_start;
							current_state = copy.current_state;
						}
						return *this;
					}
			};
		}
	}
}

#endif
