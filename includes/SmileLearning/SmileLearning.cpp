#include "SmileLearning.h"

string dynamicEMTraining(string theNet, string trainingSet){ 
	// open the data set:
	DSL_dataset ds;
	if (ds.ReadFile(trainingSet.c_str()) != DSL_OKAY) {
		cout << "Cannot read data file... exiting." << endl;
		getchar();
		exit(1);
	}
	
	// open the network:
	DSL_network net;
	if (net.ReadFile(theNet.c_str(), DSL_XDSL_FORMAT) != DSL_OKAY) {
		cout << "Cannot read network... exiting." << endl;
		getchar();
		exit(1);
	}
	
	// match the data set and the network (variables):
	vector< DSL_datasetMatch > dsMap(ds.GetNumberOfVariables());
	int varCnt = 0;  // the number of variables occuring both in the data set and the network
	for (int i = 0; i < ds.GetNumberOfVariables(); i++) {
		string id = ds.GetId(i);
		const char* idc = id.c_str();
		
		bool done = false;
		for (int j = 0; j < (int) strlen(idc) && !done; j++) {
			if (idc[j] == '_') {
				char* nodeId = (char*) malloc((j+1) * sizeof(char));
				strncpy(nodeId, idc, j);
				nodeId[j] = '\0';
				
				int nodeHdl = net.FindNode(nodeId);
				if (nodeHdl >= 0) {
					DSL_intArray orders;
					net.GetTemporalOrders(nodeHdl, orders);
					
					dsMap[varCnt].node   = nodeHdl;
					dsMap[varCnt].slice  = atoi(idc + j + 1);
					dsMap[varCnt].column = i;
					varCnt++;
					
					free(nodeId);
					done = true;
				}
			}
		}
		if (!done) {
			int nodeHdl = net.FindNode(idc);
			if (nodeHdl >= 0) {
				dsMap[varCnt].node   = nodeHdl;
				dsMap[varCnt].slice  = 0;
				dsMap[varCnt].column = i;
				varCnt++;
			}
		}
	}
	dsMap.resize(varCnt);
	
	// match the data set and the network (states):
	for (int i = 0; i < dsMap.size(); i++) {
		DSL_datasetMatch &m = dsMap[i];
		int nodeHdl = m.node;
		int colIdx = m.column;
		
		DSL_idArray* ids = net.GetNode(nodeHdl)->Definition()->GetOutcomesNames();
		const DSL_datasetVarInfo &varInfo = ds.GetVariableInfo(colIdx);
		const vector<string> &stateNames = varInfo.stateNames;
		vector<int> map(stateNames.size(), -1);
		for (int j = 0; j < (int) stateNames.size(); j++) {
			const char* id = stateNames[j].c_str();
			for (int k = 0; k < ids->NumItems(); k++) {
				char* tmpid = (*ids)[k];
				if (!strcmp(id, tmpid)) {
					map[j] = k;
				}
			}
		}
		for (int k = 0; k < ds.GetNumberOfRecords(); k++) {
			if (ds.GetInt(colIdx, k) >= 0) {
				ds.SetInt(colIdx, k, map[ds.GetInt(colIdx, k)]);
			}
		}
	}
	
	// learn parameters:
	DSL_em em;
	if (em.Learn(ds, net, dsMap) != DSL_OKAY) {
		cout << "Cannot learn parameters... exiting." << endl;
		exit(1);
	}
	
	net.WriteFile((theNet.insert(theNet.find_last_of("/") + 1, "res_")).c_str(), DSL_XDSL_FORMAT);
	
	return theNet;
}

string unrollNetwork(string theNet, int stepNumber){

	DSL_network net;
	net.ReadFile(theNet.c_str());

		// Unroll DBN for user specific timestep number
	net.SetNumberOfSlices(stepNumber);

		// Save unrolled DBN to a file.
	DSL_network unrolled;
	std::vector< int > dontcare;
	net.UnrollNetwork(unrolled,dontcare);

	string resNet = theNet.insert(theNet.find_last_of("."), "_Unrolled");
	unrolled.WriteFile(resNet.c_str());

	return resNet;
}