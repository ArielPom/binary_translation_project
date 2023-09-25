#ifndef P_DATABASE_CPP
#define P_DATABASE_CPP

/*======================================================================*/
/* included files                                                 		*/
/*======================================================================*/

#include "pin.H"
extern "C" {
#include "xed-interface.h"
}

#include <iostream>



/*======================================================================*/
/* namespace                                                     		*/
/*======================================================================*/

using namespace std;


/* ===================================================================== */
/* DATA Structures (for -prof) */
/* ===================================================================== */

// This map is used to store the RTN information for each routine
map<ADDRINT, rtn_info_t> routineMap;

// This map is used to store the call information for each call instruction
map<ADDRINT, call_info_t> callInstructionMap;

// This map is used to store the call information for each call instruction
map<ADDRINT, bbl_info_t> bblMap;

// This map is used to store the RTN address of hot routines
unordered_set<ADDRINT> topRoutines_Uset;


/* ===================================================================== */
/* DATA Structures (for -opt) */
/* ===================================================================== */

// >>>>>>>>>>>>
// This map is used to store the function calls we want to inline
map<ADDRINT, map<ADDRINT, ADDRINT>> topFunctionsMap;

// This map is used to store the function calls we want to inline
set<ADDRINT> inlinedFunctionsSet;
// >>>>>>>>>>>>

// This map is used to store the function calls we want to inline, <call_address, target_address>
map<ADDRINT, ADDRINT> calls_to_inline_Map;

// This map is used to store the routines we want to translate (for inline or reorder)
unordered_set<ADDRINT> routines_to_translate_Uset;

// This map is used to store the routines we want to commit , <rtn_address, translated_rtn_num>
map<ADDRINT, int> routines_to_commit_Map;

// This set is used to store the reorder for each routine
map<ADDRINT, vector<ADDRINT>> reorder_Map;

// This map gives us the unconditional jump instructions, <after_which_address , jump_address>
map<ADDRINT, uncond_jmp_info_t> uncond_jmps_Map;

// This map gives us conditional jump instructions target address, <address , jump_address>
map<ADDRINT, ADDRINT> reverted_cond_jmps_Map;

// Store all inst_map_t in array
instr_map_t *instr_map = NULL;

// Store all translated_rtn_t in array
translated_rtn_t *translated_rtn;

// Translation Cache containing the new code:
char *tc;


/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */

// const variables
UINT64 MAXUINT64 = uint64_t(UINT64_MAX);
const int CSV_FILE_MAX_LINES = 1000;
const int MAX_INLINE_TRANSLATION_COUNTER = 1;
const int MAX_REORDER_TRANSLATION_COUNTER = 5;
const int MAX_STRONG_CALLERS = 1;
const int COUNT_SEEN_MIN = 10;

const float HOT_THRESHOLD_PERCENTAGE = 30.0;
const int CALL_COUNT_THRESHOLD = 100;
const int INS_COUNT_THRESHOLD = 100;

// global counter
int translated_calls_counter = 0;

// Flags For Debugging 
const int DEBUG_PROF = 1;
const int DEBUG_OPT = 1;
const int SPECIAL_DEBUG_FLAG = 1;

// Files for output
ofstream OutFile;
std::ofstream* out = 0;


#endif // P_DATABASE_CPP