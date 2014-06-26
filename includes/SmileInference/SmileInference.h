
/* Standrad Included Library */
#include <map>
#include <vector>
#include <sstream>
#include <iostream>
using namespace std;

/* Core of SMILE DBN library */
#include "smile.h"
#include "smilearn.h"

namespace bn {
		// Param: 1. DBN_Model 2. The vector to store resulted name strings
	//const int getNameOfNodes(string theNet, vector< string >&);
		// Param: 1. DBN_Model 2. The vector to store resulted attribute name string 3. Which node
	map< int, string> getBNOutcomeNames(DSL_network, const string);

		// Param: 1. DBN_Model 2. The nodes want to set evidences 3. The evidences of the nodes 4. Current time step
	const int setEvidenceOfBN(DSL_network&, const vector< string >&, const vector< string >&, const int);
		// Param: 1. DBN_Model 2. The target of the node
	map< double, string > getBNOutcome(DSL_network&, const string);
};