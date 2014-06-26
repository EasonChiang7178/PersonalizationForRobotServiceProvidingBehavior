#include "SmileLearning/SmileLearning.h"

//**	The timestep of the dynamic Bayesian network **//
#define STEPNUMBER 2

#include <string>
#include <iostream>
using namespace std;

string trainingData = "../models/DBN_Model/TrainingData/HumanAttentionEstimator_Learning.txt";		// training set
string partialNet	= "../models/DBN_Model/HumanAttentionEstimator.xdsl";					// EM的時候需要用到

int main (int argc, char* argv[])
{
	string bayesianNetwork;
	string networkUnrolled;
	
	bayesianNetwork = dynamicEMTraining(partialNet, trainingData);
	networkUnrolled = unrollNetwork(bayesianNetwork, STEPNUMBER);

	cout << "> Training Succeeded!\n> Please press enter to exit...\n" << endl;
	getchar();

	return 0;
}