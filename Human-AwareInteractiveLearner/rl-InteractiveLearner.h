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
#define URGENCY		(0)
#define PARTNERNAME "仲達"
//#define IPCSERVER	"192.168.11.4"
#define IPCSERVER	"localhost"

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
			typedef enum Action {Approach,
								 //HeadShake,
								 MakeSound,
								 Rotation,
								 CallNameLow,
								 ArmWaveLow,
								 //CallNameMedium,
								 //ArmWaveMedium,
								 CallNameHigh,
								 ArmWaveHigh,
								 MoveToAroundOfPerson,
								 StraightStyle,
								 SocialStyle,
								 Joke} RobotActionSet;
			enum {actionSize = 11};

			/* Some exceptions for state and action consistancy */
				// Can be throwed in inapprioate approach (When the robot is already too closed to the person)
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
									  Contingency,
									  HighAttention} ALR;
					enum {ALRSize = 3};

					typedef enum ToA {None,
									  HasPersonNoticedMe,
									  DoesPersonLookingMe,
									  DoesPersonIgnoreMe,
									  LoseAttention,
									  Engaged} ToA;
					enum {ToASize = 6};

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
				/** Fitting the rl::concept::Simulator **/
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
					static inline double EngagedReward(int u, int pu)	{return 20 - (30 - u) * (pu - 2);}
					static inline double HaveNoticedReward(int u)		{return 7 + 1 * u;}
					static inline double HighAttentionReward(int u)		{return 10 + 1 * u;}
					static inline double LoseAttentionReward(int u)		{return -100 + u;}
					static inline double RejectReward(int u)			{return -100 + u;}
					static inline double StepReward()					{return -3;}
						// Perform the robot action
					RobotAction robot;
						// Test variable for personAttention
					int userInput;
						// For approach using, where the robot is
					//bool goFront, approached;
					int distanceToTarget;		// 0:Otherwise,	1: 3.0,	2: 2.1,	3: 1.2
					int polarRelativeTarget;	// 0:Otherwise	1: -60,	2: -30,	3: 0

					AdaptiveInterruption::ALR preAttentionState;

				public:
					/* User defined functions */
					void restart(void) {
						setPhase(phase_type());
						distanceToTarget = 0;
						polarRelativeTarget = 0;
						robot.resetToA();
#ifndef SIMULATION
						robot.toPoint(0.0, 0.0, 0.0);
						robot.turnFaceToHuman();
						Sleep(250);
#endif
							// Reset Time constriant
						interaction_start = time(NULL);
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
						/* Action! */
						switch(current_state.robotBelief) {
							case ADAPTIVE_INTERRUPTION::LoseAttention:
							case ADAPTIVE_INTERRUPTION::Engaged:
								stepGoal();
								break;

							case ADAPTIVE_INTERRUPTION::DoesPersonLookingMe:
							case ADAPTIVE_INTERRUPTION::DoesPersonIgnoreMe:
								TryToinitiatieInteraction(a);
								break;

							default:
								step(a);
								break;
						}
#ifndef SIMULATION
						/* Query the next human attention level (Observing the environment) */
						//robot.sensingATR(200);
						//preAttentionState = static_cast< ADAPTIVE_INTERRUPTION::ALR >(robot.getATR());
						//Sleep(5000);
						robot.sensingATR(200);
						preAttentionState = current_state.personAttention;
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
						robot.setAttentionLevel(userInput);
#endif
						updateToA();
						cout << "  \t<Current state> SAR: " << current_state.personAttention << ", ToA: " << current_state.robotBelief << endl
							 << "  \t<Reward received> r: " << r << endl << endl;
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

							case ArmWaveLow:
								cout << "> RobotAction: ArmWaveLow" << endl;
#ifndef SIMULATION
								robot.armWave(3);
#endif
								break;

//							case ArmWaveMedium:
//								cout << "> RobotAction: ArmWaveMedium" << endl;
//#ifndef SIMULATION
//								robot.armWave(2);
//#endif
//								break;

							case ArmWaveHigh:
								cout << "> RobotAction: ArmWaveHigh" << endl;
#ifndef SIMULATION
								robot.armWave(1);
#endif
								break;

//							case HeadShake:
//								cout << "> RobotAction: HeadShake" << endl;
//#ifndef SIMULATION
////*************************************************************TOREVISE*************************************************************************//
//								robot.headShake(15, 0);
//								robot.turnFaceToHuman();
//#endif
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
								if (polarRelativeTarget > 3) {
									cout << "> WARNING: INVALID MoveToFrontOfPerson" << endl;
									break;
								}
								if (distanceToTarget == 0) {
									//robot.movingToAroundOfHuman(0, 2.5, -90.0 + 30.0 * ++polarRelativeTarget);
									robot.movingToAroundOfHuman(0, 2.1, 0.0);
									distanceToTarget = 2;
								} else
									robot.movingToAroundOfHuman(0, 3.9 - 0.9 * distanceToTarget, 0.0);
									//robot.movingToAroundOfHuman(0, 3.9 - 0.9 * distanceToTarget, -90.0 + 30.0 * ++polarRelativeTarget);
#endif
								break;

							case Approach:
								cout << "> RobotAction: Approach" << endl;
#ifndef SIMULATION
								if (distanceToTarget++ > 3) {
									cout << "> WARNING: INVALID Approach" << endl;
									break;
								}
								robot.forwardApproach(0, 0.9);
#endif
								break;

							case MakeSound:
								cout << "> RobotAction: MakeSound" << endl;
#ifndef SIMULATION
								robot.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\models\\Soundness\\WindowsMessage.wav");
#endif
								break;

							case CallNameLow:
								cout << "> RobotAction: CallNameLow" << endl;
#ifndef SIMULATION
								robot.speaking(string(PARTNERNAME), 0.3f);
#endif
								break;

//							case CallNameMedium:
//								cout << "> RobotAction: CallNameMedium" << endl;
//#ifndef SIMULATION
//								robot.speaking(string(PARTNERNAME), 0.6f);
//#endif
//								break;

							case CallNameHigh:
								cout << "> RobotAction: CallNameHigh" << endl;
#ifndef SIMULATION
								robot.speaking(string(PARTNERNAME), 0.9f);
#endif
								break;

							default:
								std::ostringstream ostr;
								ostr << "RobotAction: stepStart(" << a << ")";
								throw BadAction(ostr.str());
						}
					}
	
					void updateToA() {
						/* Check whether the partner reject or accept the interaction */
						robot.setKeywordListened();
						if (robot.getListenContent() == "Rejected") {
							r += RejectReward(URGENCY);
							current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
							robot.resetKeyword("");
							return;
						}

						/* Check the timeout and transition to LoseAttention */
						interaction_current = time(NULL);
						if (difftime(interaction_current, interaction_start) > timeOutBound) {
							r += LoseAttentionReward(URGENCY);
							current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;

							return;
						}

						/* Update the reward */
						switch(current_state.personAttention) {
							case ADAPTIVE_INTERRUPTION::Contingency:
								if (preAttentionState == ADAPTIVE_INTERRUPTION::Neglect)
									r += HaveNoticedReward(URGENCY);
								break;

							case ADAPTIVE_INTERRUPTION::HighAttention:
								if (preAttentionState == ADAPTIVE_INTERRUPTION::Contingency || preAttentionState == ADAPTIVE_INTERRUPTION::Neglect)
									r += HighAttentionReward(URGENCY);
								break;
						}

						/* State Transition for ToA */
						switch(current_state.robotBelief) {
							case ADAPTIVE_INTERRUPTION::None:
								/* Check the person is aware of the robot and transition to HasPersonNoticedMe */
								if (robot.getContingencyFlag() > 0) {
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::HasPersonNoticedMe;
										// Extend the overall bound time
									timeOutBound += noticedTimeOutBound;
								}

								if (robot.getHighAttentionFlag() > 0) {
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::DoesPersonLookingMe;
								}

								ATR_high_start = time(NULL);
								break;

							case ADAPTIVE_INTERRUPTION::HasPersonNoticedMe:
								/* Transition to DoesPersonLookingMe */
								if (robot.getHighAttentionFlag() > 0) {
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::DoesPersonLookingMe;
								}

								/* Transition to DoesPersonIgnoreMe */
								if (robot.getContingencyFlag() > 2)
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::DoesPersonIgnoreMe;

								/* Check for losing attention */
								interaction_current = time(NULL);
								if (difftime(interaction_current, ATR_high_start) > noticedTimeOutBound) {
									r += LoseAttentionReward(URGENCY);
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
								}
								break;

							case ADAPTIVE_INTERRUPTION::DoesPersonIgnoreMe:
#ifdef SIMULATION
									// Recheck your current SAR
								cout << "> INFO: Your attention level to robot (recheck): ";
								cin >> userInput;
								current_state.personAttention = static_cast< ADAPTIVE_INTERRUPTION::ALR > (userInput);
								robot.setAttentionLevel(userInput);
#endif
								/* Lose Attetnion */
								if (robot.getLoseAttentionFlag() > 0) {
									r += LoseAttentionReward(URGENCY);
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
									break;
								}

								/* Catching the attention! */
								if (robot.getHighAttentionFlag() > 0) {
#ifndef SIMULATION
									robot.sensingPU(0);
									int pu = robot.getPU();
									robot.speaking("是時候該去運動了", 0.9f);
#else
									int pu;
									cout << "> PU detected? " << endl;
									cin >> pu;
#endif
									r += EngagedReward(URGENCY, pu);
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::Engaged;
									break;
								}

								robot.resetContingencyFlag();
								current_state.robotBelief = ADAPTIVE_INTERRUPTION::HasPersonNoticedMe;
									// Penalty on interaction time
								timeOutBound -= noticedTimeOutBound;
								break;

							case ADAPTIVE_INTERRUPTION::DoesPersonLookingMe:
#ifdef SIMULATION
									// Recheck your current SAR
								cout << "> INFO: Your attention level to robot (recheck): ";
								cin >> userInput;
								current_state.personAttention = static_cast< ADAPTIVE_INTERRUPTION::ALR > (userInput);
								robot.setAttentionLevel(userInput);
#endif
								/* Lose Attetnion */
								if (robot.getLoseAttentionFlag() > 0) {
									r += LoseAttentionReward(URGENCY);
									current_state.robotBelief = ADAPTIVE_INTERRUPTION::LoseAttention;
									break;
								}

#ifndef SIMULATION
								robot.sensingPU(0);
								int pu = robot.getPU();
								robot.speaking("是時候該去運動了", 0.9f);
#else
								int pu;
								cout << "> PU detected? ";
								cin >> pu;
#endif
								r += EngagedReward(URGENCY, pu);
								current_state.robotBelief = ADAPTIVE_INTERRUPTION::Engaged;
								break;
						}
					}

						// Ask the partner whethler he/she can be interrupted
					void TryToinitiatieInteraction(const action_type& a) {
						switch(a) {
							case StraightStyle:
#ifndef SIMULATION
								if (distanceToTarget > 3) {
									cout << "> WARNING: INVALID Approach" << endl;
									break;
								}
								robot.forwardApproach(0, 0.9);
								robot.speaking(string(PARTNERNAME) + "，我這裡有一封訊息要傳遞給你", 0.7f);

								Sleep(1000);
#else
								cout << "> Robot: " << string(PARTNERNAME) + "，我這裡有一封訊息要傳遞給你" << endl;
#endif
								break;

							case SocialStyle:
#ifndef SIMULATION
								if (distanceToTarget > 3) {
									cout << "> WARNING: INVALID Approach" << endl;
									break;
								}
								robot.forwardApproach(0, 0.9);
								robot.speaking("不好意思打擾您了，我這裡有一則訊息要轉交給您", 0.7f);
								Sleep(1000);
#else
								cout << "> Robot: 不好意思打擾您了，我這裡有一則訊息要轉交給您" << endl;
#endif
								break;

							case Joke:
#ifndef SIMULATION
								if (distanceToTarget > 3) {
									cout << "> WARNING: INVALID Approach" << endl;
									break;
								}
								robot.forwardApproach(0, 0.9);
								robot.speaking(string(PARTNERNAME) + "，我想分享一個故事給你聽，", 0.7f);
								robot.speaking("有一天小明回到家跟他爸媽說，", 0.7f);
								robot.speaking("老師問同學一個問題，只有我答的出來", 0.7f);
								robot.speaking("爸媽問，那是什麼問題", 0.7f);
								robot.speaking("誰沒有交作業", 0.9f);
								Sleep(500);
								robot.speaking("對了，我這裡有一封訊息要傳遞給你", 0.7f);
								Sleep(1000);
#else
								cout << "> Robot: " + string(PARTNERNAME) + "，我想分享一個故事給你聽，" << endl
									 << "         有一天小明回到家跟他爸媽說，" << endl
									 << "         老師問同學一個問題，只有我答的出來" << endl
									 << "         爸媽問，那是什麼問題" << endl
									 << "         誰沒有交作業" << endl
									 << "         對了，我這裡有一封訊息要傳遞給你" << endl;
#endif
								break;
						}
					}

				public:
					/** Fitting the constructors, rl::concept::Simulator **/
					Simulator(void) : current_state(), robot(IPCSERVER)
					{
#ifndef SIMULATION
						timeOutBound = 150;
						noticedTimeOutBound = 40;
						stableATRTimeBound = 3;
#else
						timeOutBound = 40;
						noticedTimeOutBound = 10;
						stableATRTimeBound = 3;
#endif
						//noticedFrame = 0;
						distanceToTarget = 0;		// 0:Otherwise,	1: 3.0,	2: 2.1,	3: 1.2
						polarRelativeTarget = 0;	// 0:Otherwise	1: -60,	2: -30,	3: 0
						interaction_start = time(NULL);
					}
					
					Simulator(const Simulator& copy) : current_state(copy.current_state), robot(IPCSERVER)
					{
						timeOutBound = copy.timeOutBound;
						noticedTimeOutBound = copy.noticedTimeOutBound;
						stableATRTimeBound = copy.stableATRTimeBound;
						//noticedFrame = 0;
						distanceToTarget = copy.distanceToTarget;
						polarRelativeTarget = copy.polarRelativeTarget;
						interaction_start = copy.interaction_start;
					}

					~Simulator(void) {}

					Simulator& operator=(const Simulator& copy) {
						if(this != &copy) {
							timeOutBound = copy.timeOutBound;
							noticedTimeOutBound = copy.noticedTimeOutBound;
							stableATRTimeBound = copy.stableATRTimeBound;
							//noticedFrame = 0;
							distanceToTarget = copy.distanceToTarget;
							polarRelativeTarget = copy.polarRelativeTarget;
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
