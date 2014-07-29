
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
string rawData = "../models/DBN_Model/TrainingData/raw_dataset/SAM_TrainingData/Data_Cleared/SocialAttentionModel_raw_Merged.txt";
	// Path to result training data
string trainingData = "../models/DBN_Model/TrainingData/SocialAttentionInferenceModel_Learning.txt";
	// Path to annotated label
string annotatedData = "../models/DBN_Model/TrainingData/SocialAttentionInferenceModel_Label.txt";
	// Path to the DBN model
string netPath = "../models/DBN_Model/SocialAttentionInferenceModel.xdsl";

/** Declration of Functions **/
	// Print the labeled dataset got from a file
void displayLabeledSet(const vector< forward_list< vector< int> > >&);
	// Search whether data already labeled
bool searchInLabeledSet(const vector< forward_list< vector< int > > >&, const vector< int >&, int &);

//=============================================================================
int main(int argc, char* argv[])
{
		// Ready? GO!
	cout << "\n		< Social Attention Model Annotation >" << endl;
	cout << "> Press ENTER to start annotating data (Social Attention Model)";
	getchar();

	/* The nodes in the network */
	vector< string > nameOfNodes;
	nameOfNodes.push_back("audioNoise");		nameOfNodes.push_back("attentionContext");
	nameOfNodes.push_back("faceDirection");		nameOfNodes.push_back("bodyDirection");	nameOfNodes.push_back("audioInput");
	nameOfNodes.push_back("robotMotionSpeed");	nameOfNodes.push_back("robotPose");		nameOfNodes.push_back("robotSoundVolume");
	nameOfNodes.push_back("attentionLevel");

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
	vector< forward_list< vector< int > > > labelDataset(3);

	fstream fannotatedLabel;
	fannotatedLabel.open(annotatedData.c_str(), ios::in);
	if (!fannotatedLabel) {
		cout << "> WARNING: Fail to open the file: " << annotatedData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	} else {
		while (fannotatedLabel.eof() == 0) {
			int observation = 0, label = 0;
			vector< int > featureVector;

			for (int i = 0; i < 8; i++) {
				fannotatedLabel >> observation;
				featureVector.push_back(observation);
			}
			fannotatedLabel >> label;

			labelDataset[label].push_front(featureVector);
		}
		fannotatedLabel.close();
	}
	/*** Main loop to annotated data ***/
	/* Prepare the training data: Insert nodes of the network */
	fstream ftrainingData;
	ftrainingData.open(trainingData.c_str(), ios::out);
	if (!ftrainingData) {
		cout << "> WARNING: Fail to open the file: " << trainingData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}

	ftrainingData << nameOfNodes[0] << " " << nameOfNodes[1];
	for (int step = 0; step < STEPNUMBER; step++) {
		stringstream ss;
		for (int feature = 2; feature < 9; feature++) {
			if (step == 0) {
				ftrainingData << " " + nameOfNodes[feature];
				continue;
			}
			string currentStep;
			ss << step; ss >> currentStep;
			ftrainingData << " " + nameOfNodes[feature] + "_" + currentStep;
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
	vector< int > audioNoise, attentionContext;
	vector< int > faceFeature, bodyFeature, voiceFeature, labelSet;
	vector< int > robotSpeed, robotPose, robotVolume;

	for (int dataNumber = 0; ; dataNumber++) {
		int observation = -1, label = -1;
		vector< int > featureVector;
			// Load raw data
		for (int i = 0; i < 8; i++) {
			frawData >> observation;
			featureVector.push_back(observation);
		}
			// End of raw data
		if (frawData.eof() == 1)
			break;

		/* If discover new data */
		//if (searchInLabeledSet(labelDataset, featureVector, label) == false) {
			stringstream ss;
			string dataNumberStr;
			char newLabel;
			ss << dataNumber; ss >> dataNumberStr;
			while (dataNumberStr.size() < 4)
				dataNumberStr.insert(0, "0");
			cv::Mat rawImg = cv::imread(rawData.substr(0, rawData.find_last_of("/") + 1) + "raw_" + dataNumberStr + ".png");
			
			if (rawImg.empty() != true)
				cv::imshow("New Raw Data", rawImg);

			/* Display the data we discovered */
			cout << "> New data!" << endl
				 << "  EN: " << featureVector[0] << ", AC: " << featureVector[1] << endl
				 << "  F:  " << featureVector[2] << ", B:  " << featureVector[3] << ", V:  " << featureVector[4] << endl
				 << "  RS: " << featureVector[5] << ", RP: " << featureVector[6] << ", RV: " << featureVector[7] << endl;
			
			cout << "> Please enter the label: ";
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

			///* Added to annotated dataset */
			//labelDataset[label].push_front(featureVector);

			///* Write to annotated data set */
			//for (auto it = featureVector.begin(); it != featureVector.end(); it++)
			//	fannotatedLabel << *it << " ";
			//fannotatedLabel << label << endl;
		//}

		/* Store the data to history repository */
		audioNoise.push_back(featureVector[0]);
		attentionContext.push_back(featureVector[1]);
		faceFeature.push_back(featureVector[2]);
		bodyFeature.push_back(featureVector[3]);
		voiceFeature.push_back(featureVector[4]);
		robotSpeed.push_back(featureVector[5]);
		robotPose.push_back(featureVector[6]);
		robotVolume.push_back(featureVector[7]);

		labelSet.push_back(label);

		/* Generate training data for DBN (HMM) */
			// If too few data
		if (dataNumber < STEPNUMBER - 1)
			continue;

			// Generate one new data
		ftrainingData << attributeNameArray[0][audioNoise[0]] << " ";
		ftrainingData << attributeNameArray[1][attentionContext[0]] << " ";
		for (int step = 0; step < STEPNUMBER; step++) {
			ftrainingData << attributeNameArray[2][faceFeature[step]] << " ";
			ftrainingData << attributeNameArray[3][bodyFeature[step]] << " ";
			ftrainingData << attributeNameArray[4][voiceFeature[step]] << " ";
			ftrainingData << attributeNameArray[5][robotSpeed[step]] << " ";
			ftrainingData << attributeNameArray[6][robotPose[step]] << " ";
			ftrainingData << attributeNameArray[7][robotVolume[step]] << " ";
			(step != STEPNUMBER - 1) ? ftrainingData << attributeNameArray[8][labelSet[step]] << " " : ftrainingData << attributeNameArray[8][labelSet[step]] << endl;
		}
			// Drop the oldest data
		audioNoise.erase(audioNoise.begin());
		attentionContext.erase(attentionContext.begin());
		faceFeature.erase(faceFeature.begin());
		bodyFeature.erase(bodyFeature.begin());
		voiceFeature.erase(voiceFeature.begin());
		robotSpeed.erase(robotSpeed.begin());
		robotPose.erase(robotPose.begin());
		robotVolume.erase(robotVolume.begin());
		labelSet.erase(labelSet.begin());
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