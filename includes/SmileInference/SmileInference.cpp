
#include "SmileInference.h"

	// Param: 1. Path to DBN_Model 2. The vector to store resulted name strings
//const int bn::getNameOfNodes(string theNet, vector< string >& bnNameOfState) {
//		// Open the network:
//	DSL_network net;
//	if (net.ReadFile(theNet.c_str(), DSL_XDSL_FORMAT) != DSL_OKAY) {
//		cout << "Cannot read network... exiting." << endl;
//		getchar();
//		exit(1);
//	}
//
//		// Clear bnNameOfState reallocating
//	vector< string >().swap(bnNameOfState);
//
//	/* Get name process */
//	for (int i = 0; i < net.GetNumberOfNodes(); i++) {
//
//	}
//}

	// Param: 1. DBN_Model 2. Which node
map< int, string > bn::getBNOutcomeNames(DSL_network net, const string nodeName) {
	int nodeHdl = net.FindNode(nodeName.c_str());
	if (nodeHdl < 0) {
		cout << "> WARNING: Node, " + nodeName + ", not found in the network..." << endl;
		return map< int, string >();
	}

		// Get the array of attributes name in nodeHdl
	DSL_idArray* ids = net.GetNode(nodeHdl)->Definition()->GetOutcomesNames();
	
	map< int, string > resultAttributes;
	for (int i = 0; i < ids->NumItems(); i++)
		resultAttributes.insert(pair< int, string >(i, (*ids)[i]));

	return resultAttributes;
}

	// Param: 1. DBN_Model 2. The nodes want to set evidences 3. The evidences of the nodes 4. Current time step
const int bn::setEvidenceOfBN(DSL_network& theDBN, const vector< string >& nodes, const vector< string >& evidences, const int timeStep) {
		// For every nodes which want to set evidences
	for (int i = 0; i < static_cast< int >(nodes.size()); i++) {
		string curNode = nodes[i];
			// If timeStep > 0, add "_0", "_1", "_2".. after the node names
		if (timeStep > 0) {
			stringstream itoaObj;
			itoaObj << timeStep;
			curNode += "_" + itoaObj.str();
		}

		DSL_node *thisNode = theDBN.GetNode(theDBN.FindNode(curNode.c_str()));
		int numOfOutcomes = thisNode->Definition()->GetNumberOfOutcomes();
		DSL_idArray *idArray = thisNode->Definition()->GetOutcomesNames();
			// For all observations of the current node
		for (int j = 0; j < numOfOutcomes; j++) {
				// If the evidence exists
			if (strcmp((*idArray)[j], evidences[i].c_str()) == 0) {
				thisNode->Value()->SetEvidence(j);
				//cout << "Node: " << curNode << "\t\tEvidence: " << evidences[i] << endl;
				break;
			}
				// Error occurred! Evidences didn't exist!
			if (j == numOfOutcomes - 1) {
				cout << "> WARNING: Evidence did not exist!" << endl;
				exit(1);
			}
		}
	}

	return 0;
}

	// Param: 1. DBN_Model 2. The target of the node
map< double, string > bn::getBNOutcome(DSL_network& theDBN, const string targetNode) {
	map< double, string > tempResult;
		// Get the results of the node
	DSL_node* target = theDBN.GetNode(theDBN.FindNode(targetNode.c_str()));
	DSL_idArray *idArray = target->Definition()->GetOutcomesNames();
	DSL_Dmatrix *theMatrix = target->Value()->GetMatrix();
	int matrixSize = theMatrix->GetSize();
		// For every outcome
	for (int i = 0; i < matrixSize; i++)
		tempResult.insert(pair< double, string >(theMatrix->Subscript(i), (*idArray)[i]));
	
	return tempResult;
}