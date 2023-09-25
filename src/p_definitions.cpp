#ifndef P_DEFINITIONS_CPP
#define P_DEFINITIONS_CPP

/*======================================================================*/
/* included files                                                 		*/
/*======================================================================*/

#include "pin.H"
extern "C" {
#include "xed-interface.h"
}

#include <iostream>
#include "p_definitions.cpp"

/*======================================================================*/
/* namespace                                                     		*/
/*======================================================================*/

using namespace std;

/*======================================================================*/
/* colors	                                                     		*/
/*======================================================================*/

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"


/*======================================================================*/
/* Strcuts and Objects                                                 	*/
/*======================================================================*/

typedef struct {
    ADDRINT         head_address;
	ADDRINT         tail_address;
	UINT64			countSeen;
	UINT64 			numIns;
	bool 			HasFallThrough;
	UINT64			countFallTrhough;
	UINT64			countJumped;
	ADDRINT 		fallThroughAAdress;
	ADDRINT 		jumpAddress;
	map<ADDRINT, UINT64> nextBblMap; // counts for each BBL which BBLs it called
	ADDRINT 		rtnAddress;
} bbl_info_t;


// This struct is used to store the RTN information
typedef struct {
    ADDRINT         address;
    string          name;
    UINT64          callCount;
    UINT64          insCount;
    UINT32          numIns;
	bool 			not_safe_to_inline;
	UINT64 			positiveRbpAccess;
	UINT64 			negativeRspAccess;
	unordered_set<ADDRINT> retMap; // counts different ret ins
	unordered_set<ADDRINT> callingMap; // counts different call ins
	unordered_set<ADDRINT> directJumpsMap; // counts different direct jumps outside the scope of the function
	unordered_set<ADDRINT> indirectJumpsMap; // counts different indirect jumps outside the scope of the function
	map<ADDRINT, UINT64> callersMap; // counts for each caller of this routine how many times it got called
	set<ADDRINT> bblsSet; // to keep bbls for each routine and sort them in the end
} rtn_info_t;


// This struct is used to store loop information
typedef struct {
	ADDRINT         address;
 	ADDRINT         targetAddress;
	UINT64          countSeen;
	string			originRoutineName;
	string 			targetRoutineName;
    rtn_info_t*    	originRtn;
	rtn_info_t*    	targetRtn;
} call_info_t;


// instruction map with an entry for each new instruction:
typedef struct { 
	ADDRINT orig_ins_addr;
	ADDRINT new_ins_addr;
	ADDRINT orig_targ_addr;
	bool hasNewTargAddr;
	char encoded_ins[XED_MAX_INSTRUCTION_BYTES];
	xed_category_enum_t category_enum;
	unsigned int size;
	int targ_map_entry;
} instr_map_t;


// Tables of all candidate routines to be translated:
typedef struct { 
	ADDRINT rtn_addr; 
	USIZE rtn_size;
	int instr_map_entry;   // negative instr_map_entry means routine does not have a translation.
	bool isSafeForReplacedProbe;	
} translated_rtn_t;

// Info abour unconditional jump instruction
typedef struct {
	ADDRINT after_which_address;
	ADDRINT jump_address;
	xed_decoded_inst_t xedd;
} uncond_jmp_info_t;



#endif // P_DEFINITIONS_CPP