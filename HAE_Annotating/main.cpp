
/* Standrad Included Library */
#include <iostream>
#include <fstream>
#include <sstream>
#include <forward_list>
#include <list>
#include <conio.h>			// To use getch(), kbhit()
using namespace std;

/* Third-party Library */
#include "SmileInference\SmileInference.h"
#include <opencv/highgui.h>

//** Problem Dependent Variable Setting **//
	// The timestep of the dynamic Bayesian network
#define STEPNUMBER	2

/** Declration of Variables **/
	// Path to store the raw data
string rawData = "../models/DBN_Model/TrainingData/DataFrame/HumanAttentionEstimator_raw.txt";
	// Path to result training data
string trainingData = "../models/DBN_Model/TrainingData/HumanAttentionEstimator_Learning.txt";
	// Path to annotated label
string annotatedData = "../models/DBN_Model/TrainingData/HumanAttentionEstimator_Label.txt";
	// Path to the DBN model
string netPath = "../models/DBN_Model/HumanAttentionEstimator.xdsl";

/** Declration of Functions **/
	// Print the labeled dataset got from a file
void displayLabeledSet(const vector< forward_list< vector< int> > >&);
	// Search whether data already labeled
bool searchInLabeledSet(const vector< forward_list< vector< int > > >&, const vector< int >&, int &);

//=============================================================================
int main(int argc, char* argv[])
{
		// Ready? GO!
	cout << "\n		< Human Attention Estimator Annotation >" << endl;
	cout << "> Press ENTER to start annotating data (Human Attention Estimator)";
	getchar();

	/* The nodes in the network */
	vector< string > nameOfNodes;
	nameOfNodes.push_back("faceDirection");		nameOfNodes.push_back("bodyDirection");
	nameOfNodes.push_back("voiceDetection");	nameOfNodes.push_back("attentionLevel");

	/* Extract each nodes' attributes name in the network */
	DSL_network net;
	if (net.ReadFile(netPath.c_str(), DSL_XDSL_FORMAT) != DSL_OKAY) {
		cout << "> WARNING: Cannot read network... exiting." << endl;
		getchar();
		exit(1);
	}

	vector< map< int, string> > attributeNameArray;
	for (int i = 0; i < static_cast< int>(nameOfNodes.size()); i++)
		attributeNameArray.push_back(bn::getBNOutcomeNames(net, nameOfNodes[i]));

	/* Load the annotated label */
	fstream fannotatedLabel;
	fannotatedLabel.open(annotatedData.c_str(), ios::in);
	if (!fannotatedLabel) {
		cout << "> WARNING: Fail to open the file: " << rawData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}

	vector< forward_list< vector< int > > > labelDataset(4);
	while (fannotatedLabel.eof() == 0) {
		int observation = 0, label = 0;
		vector< int > featureVector;

		for (int i = 0; i < 3; i++) {
			fannotatedLabel >> observation;
			featureVector.push_back(observation);
		}
		fannotatedLabel >> label;

		labelDataset[label].push_front(featureVector);
	}
	fannotatedLabel.close();

	/*** Main loop to annotated data ***/
	/* Prepare the training data: Insert nodes of the network */
	fstream ftrainingData;
	ftrainingData.open(trainingData.c_str(), ios::out);
	if (!ftrainingData) {
		cout << "> WARNING: Fail to open the file: " << trainingData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}

	for (int i = 0; i < static_cast< int>(nameOfNodes.size()); i++) {
		stringstream ss;
		(i == 0) ? ftrainingData << nameOfNodes[i] : ftrainingData << " " + nameOfNodes[i];
		for (int j = 1; j < STEPNUMBER; j++) {
			string currentStep;
			ss << j; ss >> currentStep;
			ftrainingData << " " + nameOfNodes[i] + "_" + currentStep;
			ss.str(""); ss.clear();
		}
	}
	ftrainingData << endl;

	/* Open the raw data set*/
	fstream frawData;
	frawData.open(rawData.c_str(), ios::in);
	if (!frawData) {
		cout << "> WARNING: Fail to open the file: " << rawData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}
	fannotatedLabel.open(annotatedData.c_str(), ios::app);

	/* For each raw data */
	list< int > faceFeature, bodyFeature, voiceFeature, labelSet;

	for (int dataNumber = 0; ; dataNumber++) {
		int observation = -1, label = -1;
		vector< int > featureVector;
			// Load raw data
		for (int i = 0; i < 3; i++) {
			frawData >> observation;
			featureVector.push_back(observation);
		}
			// End of raw data
		if (frawData.eof() == 1)
			break;

		/* If discover new data */
		if (searchInLabeledSet(labelDataset, featureVector, label) == false) {
			stringstream ss;
			string dataNumberStr;
			char newLabel;
			ss << dataNumber; ss >> dataNumberStr;
			cv::Mat rawImg = cv::imread(rawData.substr(0, rawData.find_last_of("/") + 1) + "raw_" + dataNumberStr + ".png");
			
			if (rawImg.empty() != true)
				cv::imshow("New Raw Data", rawImg);

			cout << "> Data: (";
			for (auto it = featureVector.begin(); it != featureVector.end(); it++)
				(it != featureVector.end() - 1) ? cout << (*it) << "," : cout << (*it);
			cout << "), Please enter the label: ";
			if (rawImg.empty() != true)
				newLabel = cv::waitKey(0);
			else
				newLabel = getch();
			cout << newLabel << endl;

			if (rawImg.empty() != true)
				cv::destroyWindow("New Raw Data");

			if (newLabel == 'n') {
				cout << "> NOTICE: Skip this data!" << endl;
				continue;
			}

			ss.str(""); ss.clear();
			ss << newLabel; ss >> label;
			if (label < 0 || label > 4) {
				cout << "> ERROR: Please enter the correct label" << endl;
				continue;
			}

			/* Added to annotated dataset */
			labelDataset[label].push_front(featureVector);

			/* Write to annotated data set */
			for (auto it = featureVector.begin(); it != featureVector.end(); it++)
				fannotatedLabel << *it << " ";
			fannotatedLabel << label << endl;
		}

		/* Store the data to history repository */
		faceFeature.push_back(featureVector[0]);
		bodyFeature.push_back(featureVector[1]);
		voiceFeature.push_back(featureVector[2]);
		labelSet.push_back(label);

		/* Generate training data for DBN (HMM) */
			// If too few data
		if (dataNumber < STEPNUMBER - 1)
			continue;

			// Generate one new data
		for (auto it = faceFeature.begin(); it != faceFeature.end(); it++)
			ftrainingData << attributeNameArray[0][*it] << " ";
		for (auto it = bodyFeature.begin(); it != bodyFeature.end(); it++)
			ftrainingData << attributeNameArray[1][*it] << " ";
		for (auto it = voiceFeature.begin(); it != voiceFeature.end(); it++)
			ftrainingData << attributeNameArray[2][*it] << " ";
		for (auto it = labelSet.begin(); it != labelSet.end(); it++) {
			auto itNext = it;
			(++itNext != labelSet.end()) ? ftrainingData << attributeNameArray[3][*it] << " " : ftrainingData << attributeNameArray[3][*it] << endl;
		}
			// Drop the oldest data
		faceFeature.pop_front(); bodyFeature.pop_front(); voiceFeature.pop_front(); labelSet.pop_front();
	}

	fannotatedLabel.close();
	ftrainingData.close();
	frawData.close();
	cout << "\n> Annotation Succeeded!\n> Please press ENTER to left..." << endl;
	getchar();

	return 0;
}

//=============================================================================
	// Print the labeled dataset got from a file
void displayLabeledSet(const vector< forward_list< vector< int> > >& labelDataset) {
	/* Test annotated dataset */
	int i = 0;
	for (auto it = labelDataset.begin(); it != labelDataset.end(); it++, i++) {
		cout << i << ":";
		for (auto itFL = (*it).begin(); itFL != (*it).end(); itFL++) {
			cout << "(";
			for (auto itF = (*itFL).begin(); itF != (*itFL).end(); itF++)
				(itF != itFL->end()) ? cout << (*itF) << "," : cout << (*itF);
			(itFL != it->end()) ? cout << ") " : cout << ")";
		}
		cout << endl;
	}
}
	// Search whether data already labeled
bool searchInLabeledSet(const vector< forward_list< vector< int > > >& labelDataset,const vector< int >& testData, int &label) {
	for (int i = 0; i < static_cast< int >(labelDataset.size()); i++) {
		for (auto itFL = labelDataset[i].begin(); itFL != labelDataset[i].end(); itFL++) {
			if (testData != *itFL)
				continue;
				// Label Found!
			label = i;

			cout << "> Load label from set: " << i << ":";
			cout << "(";
			for (auto it = testData.begin(); it != testData.end(); it++)
				(it != testData.end() - 1) ? cout << (*it) << "," : cout << (*it);
			cout << ")" << endl;
			return true;
		}
	}
	return false;
}