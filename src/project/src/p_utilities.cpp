#ifndef P_UTILITIES_CPP
#define P_UTILITIES_CPP

/*======================================================================*/
/* included files                                                 		*/
/*======================================================================*/

#include "pin.H"
extern "C" {
#include "xed-interface.h"
}

#include <iostream>

#include "p_definitions.cpp"
#include "p_database.cpp"

/*======================================================================*/
/* Functions                                                     		*/
/*======================================================================*/


/**
 * @brief This function is used give back the reverted iclass
 * @note used in -opt mode
 */
xed_iclass_enum_t get_reverted_iclass_enum(xed_iclass_enum_t iclass_enum)
{
	xed_iclass_enum_t retverted_iclass = XED_ICLASS_INVALID;

	switch (iclass_enum)
	{
		case XED_ICLASS_JB:
			retverted_iclass = XED_ICLASS_JNB;		
			break;

		case XED_ICLASS_JBE:
			retverted_iclass = XED_ICLASS_JNBE;
			break;

		case XED_ICLASS_JL:
			retverted_iclass = XED_ICLASS_JNL;
			break;
	
		case XED_ICLASS_JLE:
			retverted_iclass = XED_ICLASS_JNLE;
			break;

		case XED_ICLASS_JNB: 
			retverted_iclass = XED_ICLASS_JB;
			break;

		case XED_ICLASS_JNBE: 
			retverted_iclass = XED_ICLASS_JBE;
			break;

		case XED_ICLASS_JNL:
			retverted_iclass = XED_ICLASS_JL;
			break;

		case XED_ICLASS_JNLE:
			retverted_iclass = XED_ICLASS_JLE;
			break;

		case XED_ICLASS_JNO:
			retverted_iclass = XED_ICLASS_JO;
			break;

		case XED_ICLASS_JNP: 
			retverted_iclass = XED_ICLASS_JP;
			break;

		case XED_ICLASS_JNS: 
			retverted_iclass = XED_ICLASS_JS;
			break;

		case XED_ICLASS_JNZ:
			retverted_iclass = XED_ICLASS_JZ;
			break;

		case XED_ICLASS_JO:
			retverted_iclass = XED_ICLASS_JNO;
			break;

		case XED_ICLASS_JP: 
			retverted_iclass = XED_ICLASS_JNP;
			break;

		case XED_ICLASS_JS: 
			retverted_iclass = XED_ICLASS_JNS;
			break;

		case XED_ICLASS_JZ:
			retverted_iclass = XED_ICLASS_JNZ;
			break;

		default:
			retverted_iclass = XED_ICLASS_INVALID;
			break;
	}

	return retverted_iclass;
}

/**
 * @brief This function is used to print two instructions in a nice format
 * @note used in -opt mode
 */
void print_original_and_new_instr(xed_decoded_inst_t old_xedd, xed_decoded_inst_t new_xedd, ADDRINT address)
{
	char old_xedd_buffer[2048];
	char new_xedd_buffer[2048];		

	xed_format_context(XED_SYNTAX_INTEL, &old_xedd, old_xedd_buffer, 2048, address, 0, 0);
	xed_format_context(XED_SYNTAX_INTEL, &new_xedd, new_xedd_buffer, 2048, address, 0, 0);

	cerr << "orig instr: " << hex << address << " " << old_xedd_buffer << endl;
	cerr << "new  instr: " << hex << address << " " << new_xedd_buffer << endl;
}


/**
 * @brief This function Restores the routines map we want to reorder
 * @note used in -opt mode
 */
void restore_routines_map()
{
	// Open the profile.csv file to read
    ifstream file("profile.csv");
    if (!file.is_open())
	{
        cout << "Failed to open the CSV file. did you first create it using -prof flag?" << endl;
        return;
    }

    string line, field;
	vector<string> row;
	bool skip_first_line = true;

	if(DEBUG_OPT) cout << endl << GREEN << ">>>>> restoring routines to reorder <<<<<" << RESET << endl;
    while (getline(file, line))
	{
		row.clear();
        istringstream str(line);

        if (skip_first_line)
		{
			skip_first_line = false;
			continue;
		}

		if (line.empty())
			return;

		if(DEBUG_OPT) cout << line << endl;

        while (getline(str, field, ','))
		{
			field.erase(std::remove_if(field.begin(), field.end(), ::isspace), field.end());
            row.push_back(field);
        }
		
		ADDRINT rtn_address		 	= stoul(row[0].erase(0, 2), nullptr, 16);
        string 	rtn_name 			= row[1];

		if(DEBUG_OPT) cout << "routine [" << rtn_name << "] is added to so it will be reordered" << endl;

		routines_to_translate_Uset.insert(rtn_address);
		routines_to_commit_Map[rtn_address] = 0;
    }

    file.close();

	if(DEBUG_OPT) cout << "-----------------------------------------" << endl;
}

/**
 * @brief This function Restores the call map we want to inline
 * @note used in -opt mode
 */
void restore_call_map()
{
	// Open the profile.csv file to read
    ifstream file("profile.csv");
    if (!file.is_open())
	{
        cout << "Failed to open the CSV file. did you first create it using -prof flag?" << endl;
        return;
    }

    string line, field;
	vector<string> row;
	bool keep_skipping = true;

	int translation_counter = 0;

	if(DEBUG_OPT) cout << endl << GREEN << ">>>>> restoring calls to inline <<<<<" << RESET << endl;
    while (getline(file, line))
	{
		row.clear();
        istringstream str(line);


		// skip to the first line
        if (line.find("StrongestCallerAddress") == string::npos && keep_skipping)
			continue;

        if (line.find("StrongestCallerAddress") != string::npos)
		{
			keep_skipping = false;
			continue;
		}
		// return if reached empty line seperating reordering info
		if (line.empty())
			return;

		if(DEBUG_OPT) cout << line << endl;

		// Split the line into a vector of strings (removing spaces)
        while (getline(str, field, ','))
		{
			field.erase(std::remove_if(field.begin(), field.end(), ::isspace), field.end());
            row.push_back(field);
        }
		
		ADDRINT caller_rtn_address 	= stoul(row[0].erase(0, 2), nullptr, 16);
        ADDRINT call_ins_address 	= stoul(row[1].erase(0, 2), nullptr, 16);
		string call_rtn_name 		= row[2];
		ADDRINT target_address 		= stoul(row[4].erase(0, 2), nullptr, 16);
		string target_rtn_name 		= row[6];

		if(DEBUG_OPT) cout << "routine [" << call_rtn_name << "] calling routine [" << target_rtn_name << "]" << endl
							<< "caller_rtn_address: 0x" << hex << caller_rtn_address
							<< ", call_ins_address: 0x" << hex << call_ins_address
							<< ", targetAdress: 0x" 	<< hex << target_address << endl;
		
		if(DEBUG_OPT) cout << RED << "Adding to routines_to_translate_Uset [" << target_rtn_name << "] with call [0x" << hex << call_ins_address << "]" << RESET << endl << endl;
		
		routines_to_translate_Uset.insert(caller_rtn_address);
		routines_to_translate_Uset.insert(target_address);

		routines_to_commit_Map[caller_rtn_address] = 0;

		calls_to_inline_Map[call_ins_address] = target_address;

		translation_counter++;
		if(DEBUG_OPT) cout << RED << "translation_counter = " << translation_counter << RESET << endl << endl;
    }

    file.close();

	if(DEBUG_OPT) cout << "-----------------------------------------" << endl;
}


/**
 * @brief Restores the reorder map for each routine
 * @note used in -opt mode
 */
void restore_reorder_map()
{
	// Open the profile.csv file to read
    ifstream file("profile.csv");
    if (!file.is_open())
	{
        cout << "Failed to open the CSV file. did you first create it using -prof flag?" << endl;
        return;
    }

	// Read each line of the file
    string line, field;
	vector<string> row;

	bool keep_skipping = true;

	if(DEBUG_OPT) cout << endl << GREEN << ">>>>> creating reorder_Map <<<<<" << RESET << endl;
    while (std::getline(file, line))
	{
		row.clear();
        istringstream str(line);

		// skip to the first line
        if (line.find("orderedBBLS") == string::npos && keep_skipping)
			continue;

        if (line.find("orderedBBLS") != string::npos)
		{
			keep_skipping = false;
			continue;
		}

		// Split the line into a vector of strings (removing spaces)
        while (std::getline(str, field, ','))
		{
			field.erase(std::remove_if(field.begin(), field.end(), ::isspace), field.end());
            row.push_back(field);
        }
		
		ADDRINT rtn_address 	= stoul(row[0].erase(0, 2), nullptr, 16);
		string rtn_name 		= row[1];

		if(DEBUG_OPT) cout << "adding routine [" << rtn_name << "] at address [0x" << hex << rtn_address << "]" << endl;

		for (size_t i = 2; i < row.size()-1; ++i)
		{
			ADDRINT bbl_Address = stoul(row[i].erase(0, 2), nullptr, 16);
			reorder_Map[rtn_address].push_back(bbl_Address);
			if(DEBUG_OPT) 
			{
				cout << "	[" << dec << i-2 << "] " << hex << bbl_Address;
				if (((i-1) % 10 == 0) || (i == row.size() - 2)) cout << endl;
			}
		}
		if(DEBUG_OPT) cout << "-----------------------------------------" << endl;
    }
    file.close();
	if(DEBUG_OPT) cout << "-----------------------------------------" << endl;
}


/**
 * @brief Creates the reorder map for each routine
 * @note used in -prof mode
 */
vector<ADDRINT> create_ordered_BBL_vector(const rtn_info_t& rtnInfo)
{

	if(DEBUG_PROF) cout << "Routine Name [" << rtnInfo.name
						<< "] Routine Address [0x" << hex << rtnInfo.address
						<< "] NumIns " << rtnInfo.numIns << endl;
	
	set<ADDRINT> bblsSet_copy = rtnInfo.bblsSet;

    vector<ADDRINT> orderedBBLs_vec;

    if(bblsSet_copy.empty())
	{
		if(DEBUG_PROF) cout << "empty bblsSet for this routine" << endl;
		if(DEBUG_PROF) cout << "-----------------------------------------" << endl;
		return orderedBBLs_vec;
	}

	int insCount = 0;

	// Get first bbl in the set
	ADDRINT currentBBL_address = *bblsSet_copy.begin();
	bbl_info_t& currentBBL = bblMap[currentBBL_address];
	bblsSet_copy.erase(currentBBL_address);
    orderedBBLs_vec.push_back(currentBBL_address);
	insCount += currentBBL.numIns;

	if(DEBUG_PROF) cout << "	Started from bbl " << hex << currentBBL_address << endl;

    // Keep ordering BBLs until the set is empty
    while(!bblsSet_copy.empty())
	{
		// if currentBBL.nextBblMap is empty than go to the next one in the original set
		if(currentBBL.nextBblMap.size() == 0)
		{
			currentBBL_address = *bblsSet_copy.begin();;
			currentBBL = bblMap[currentBBL_address];
			bblsSet_copy.erase(currentBBL_address);
			orderedBBLs_vec.push_back(currentBBL_address);
			insCount += currentBBL.numIns;
			if(DEBUG_PROF) cout << "			next bbl " << hex << currentBBL_address << endl;
			continue;
		}

        ADDRINT nextBBL_address = 0;
        UINT64 maxCount = 0;

        // Print the currentBBL.nextBblMap
		for (const auto& pair : currentBBL.nextBblMap)
		{
			ADDRINT bbl_address = pair.first;
			UINT64 count = pair.second;
			if(bblsSet_copy.find(bbl_address) != bblsSet_copy.end() && count > maxCount)
			{
				maxCount = count;
				nextBBL_address = bbl_address;
			}
			if(DEBUG_PROF) cout << "							BBL: " << hex << bbl_address << ", count: " << dec << count << endl;
		}

		// if we didnt find a nextBbl which is in the bblsSet_copy, just insert the first one of bblsSet_copy
		if(nextBBL_address == 0)
		{
			currentBBL_address = *bblsSet_copy.begin();;
			currentBBL = bblMap[currentBBL_address];
			bblsSet_copy.erase(currentBBL_address);
			orderedBBLs_vec.push_back(currentBBL_address);
			insCount += currentBBL.numIns;
			if(DEBUG_PROF) cout << "			next bbl " << hex << currentBBL_address << endl;
			continue;
		}

		// assign nextBBL to currentBBL and remove it from the original set
		currentBBL_address = nextBBL_address;
		currentBBL = bblMap[nextBBL_address];
		bblsSet_copy.erase(nextBBL_address);
		orderedBBLs_vec.push_back(currentBBL_address);
		insCount += currentBBL.numIns;
		if(DEBUG_PROF) cout << "			next bbl " << hex << nextBBL_address << endl;
    }

	int originalIndex = 0;
	// for (const ADDRINT bbl : orderedBBLs_vec)
	for (size_t i = 0; i < orderedBBLs_vec.size(); ++i)
	{
		ADDRINT bbl = orderedBBLs_vec[i];
		originalIndex = distance(rtnInfo.bblsSet.begin(), rtnInfo.bblsSet.find(bbl));
		if(DEBUG_PROF) cout << "[" << dec << originalIndex
							<< "] " << hex << bbl << endl;
	}

	if(DEBUG_PROF) cout << "Total Ins : " << dec << insCount << endl; // doesnt add up 
    if(DEBUG_PROF) cout << "-----------------------------------------" << endl;

    return orderedBBLs_vec;
}

/**
 * @brief This function is used to print the routineMap to the csv output file
 * @note used in -prof mode
 */
void write_hottest_routines_to_csv()
{

	// Create a vector of pointers to the rtn_info_t objects
    vector<rtn_info_t*> rtnVec;
    for (auto& it : routineMap)
        	rtnVec.push_back(&(it.second));

    // Sort the vector based on insCount
    sort(rtnVec.begin(), rtnVec.end(),
         [](rtn_info_t* a, rtn_info_t* b) { return a->insCount > b->insCount; });

	// Write titles for each column
	OutFile << "rtnAddress" 	<< ", "
			<< "rtnName" 		<< ", "
			<< "countSeen" 		<< ", "
            << "insCount" 		<< endl;


    // Write top hottest loops information to the output file
	size_t iterations = std::min(rtnVec.size(), static_cast<size_t>(MAX_REORDER_TRANSLATION_COUNTER));
	if(DEBUG_PROF) cout << "routineMap size is " << routineMap.size() << " , Printing only " << iterations << endl;

    for (size_t i = 0; i < iterations; i++)
    {
        rtn_info_t& rtnInfo = *rtnVec[i];
        UINT64 insCount = rtnInfo.insCount;
        UINT64 callCount = 0;
        for (const auto& caller : rtnInfo.callersMap)
            callCount += caller.second;

        if(insCount < INS_COUNT_THRESHOLD || callCount < CALL_COUNT_THRESHOLD)
        {
            continue;
        }


		// Write the loop information to the output file
		OutFile	<< hex << "0x" << rtnInfo.address << ", "
                << rtnInfo.name << ", "
                << dec << callCount << ", "
                << dec << insCount << endl;

		topRoutines_Uset.insert(rtnInfo.address);
    }

    OutFile << endl;
}

/**
 * @brief This function is used to print the callInstructionMap to the csv output file
 * @note used in -prof mode
 */
void write_hottest_calls_to_csv()
{
	// Create a vector of pointers to the call_info_t objects
    vector<call_info_t*> callVec;
    for (auto& it : callInstructionMap)
        	callVec.push_back(&(it.second));

    // Sort the vector based on countSeen
    sort(callVec.begin(), callVec.end(),
         [](call_info_t* a, call_info_t* b) { return a->countSeen > b->countSeen; });

	// Write titles for each column
	OutFile << "originRtnAddress" 		<< ", "
			<< "callAddress" 			<< ", "
			<< "callerRtnName" 			<< ", "
			<< "countSeen" 				<< ", "
			<< "targetAdress" 			<< ", "
			<< "targetRtnName" 			<< ", "
			<< "targetInsCount" 		<< ", "
			<< "StrongestCallerAddress" << endl;

	unordered_set<ADDRINT> printedRoutines_Uset;

    // Write top hottest loops information to the output file
	size_t iterations = std::min(callVec.size(), static_cast<size_t>(CSV_FILE_MAX_LINES));
	if(DEBUG_PROF) cout << "callInstructionMap size is " << callInstructionMap.size() << " , Printing only " << iterations << endl;

    for (size_t i = 0; i < iterations; i++)
    {
        call_info_t& callInfo = *callVec[i];
		rtn_info_t& targetRtnInfo = *(callInfo.targetRtn);
        if(DEBUG_PROF) cout << "checking call from routine named " << callInfo.originRtn->name << endl;

		// Check if the routine to be inlined is allready printed by its strongest caller
		if(printedRoutines_Uset.find(callInfo.targetAddress) != printedRoutines_Uset.end())
		{
			if(DEBUG_PROF) cout << "target routine [" << callInfo.targetRoutineName << "] is allready printed by the strongest caller" << endl;
			continue;
		}

		// check if the call is_safe_to_inline
        if(targetRtnInfo.not_safe_to_inline && targetRtnInfo.retMap.size() <= 1)
			continue;
			
		// check if the call is seen at least COUNT_SEEN_MIN
		if(callInfo.countSeen < COUNT_SEEN_MIN)
			continue;

		// Calculate the HighestCallCounter (most dominant caller)
		ADDRINT StrongestCallerAddress = 0;
		UINT64 HighestCallCounter = 0;
		for (const auto& caller : targetRtnInfo.callersMap)
		{	
			ADDRINT address = caller.first;
			UINT64 counter = caller.second;
			if(counter > HighestCallCounter)
			{
				HighestCallCounter = counter;
				StrongestCallerAddress = address;
			}
		}

		// Calculate the number of strong callers
		int StrongCallersCounter = 0;
		UINT64 COUNTER_THRESHOLD = (HOT_THRESHOLD_PERCENTAGE / 100.0) * HighestCallCounter;
		for (const auto& caller : targetRtnInfo.callersMap)
		{
				UINT64 counter = caller.second;
				if(counter > COUNTER_THRESHOLD)
					StrongCallersCounter += 1 ;
		}
		
		// If the loop has more than MAX_STRONG_CALLERS, dont write it to the output file
		if(StrongCallersCounter > MAX_STRONG_CALLERS)
		{
			if(DEBUG_PROF) cout << "Routine has " << StrongCallersCounter << " strong callers, not added to csv file" << endl;
			continue;
		}
		
		// If we reached the max number of routines to inline, stop
		if(printedRoutines_Uset.size() > MAX_INLINE_TRANSLATION_COUNTER)
		{
			break;
		}
		// Write the loop information to the output file
		rtn_info_t& rtnInfo = *(callInfo.originRtn);
		OutFile	<< hex << "0x" << rtnInfo.address << ", "
				<< hex << "0x" << callInfo.address << ", "
				<< callInfo.originRoutineName << ", "
				<< dec << callInfo.countSeen << ", "
				<< hex << "0x" << callInfo.targetAddress << ", "
				<< callInfo.targetRoutineName << ", "
				<< dec << targetRtnInfo.insCount << ", "
				<< hex << "0x" << StrongestCallerAddress << endl;

		printedRoutines_Uset.insert(rtnInfo.address);
		printedRoutines_Uset.insert(callInfo.targetAddress);

		topRoutines_Uset.insert(rtnInfo.address);
		topRoutines_Uset.insert(callInfo.targetAddress);
    }
    OutFile << endl;
}

/**
 * @brief This function is used to print the reordered routines to the csv output file
 * @note used in -prof mode
 */
void write_routines_reorder_to_csv()
{
	// Write titles for each column for the bbl sorting for each routine
	OutFile << "rtnAddress" 			<< ", "
			<< "rtnName" 				<< ", "
			<< "orderedBBLS" 			<< endl;

    if(DEBUG_PROF) cout << endl;

	// Loop through topRoutines_Uset and find routine addresses
    for (const auto& rtn_address : topRoutines_Uset)
	{
		rtn_info_t& rtnInfo = routineMap[rtn_address];

		if(rtnInfo.address == 0)
			continue;

    	cout << "rtn set list size " << int(rtnInfo.bblsSet.size()) << endl;
		OutFile	<< hex << "0x" << rtnInfo.address << ", " << rtnInfo.name << ", ";

        vector<ADDRINT> orderedBBLs  = create_ordered_BBL_vector(rtnInfo);
		for (const ADDRINT bbl_address : orderedBBLs)
			OutFile	<< hex << "0x" << bbl_address << ", ";

		OutFile << endl;
    }
}

/**
 * @brief This function is used to print the settings for the program
 * @note used in -opt mode
 */

void print_program_settings()
{
	cout << "CSV_FILE_MAX_LINES = " 			<< CSV_FILE_MAX_LINES 				<< endl
		 << "MAX_INLINE_TRANSLATION_COUNTER = " << MAX_INLINE_TRANSLATION_COUNTER	<< endl
		 << "MAX_STRONG_CALLERS = " 			<< MAX_STRONG_CALLERS				<< endl
		 << "HOT_THRESHOLD_PERCENTAGE = "		<< HOT_THRESHOLD_PERCENTAGE 		<< endl
		 << "Num of translated calls = "		<< translated_calls_counter 		<< endl;
}

#endif // P_UTILITIES_CPP
