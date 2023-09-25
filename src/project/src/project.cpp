/*BEGIN_LEGAL 
Intel Open Source License 

Copyright (c) 2002-2011 Intel Corporation. All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.  Redistributions
in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.  Neither the name of
the Intel Corporation nor the names of its contributors may be used to
endorse or promote products derived from this software without
specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
END_LEGAL */

/* ===================================================================== */
/*! @file
 * This probe pintool generates translated code of routines, places them in an allocated TC 
 * and patches the orginal code to jump to the translated routines.
 */


/*======================================================================*/
/* included files                                                 		*/
/*======================================================================*/
#include "pin.H"
extern "C" {
#include "xed-interface.h"
}

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <errno.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <values.h>

#include <string>
#include <map>
#include <set>
#include <vector>
#include <unordered_set>
#include <vector>
#include <limits>
#include <regex>

#include "p_definitions.cpp"
#include "p_utilities.cpp"


/*======================================================================*/
/* commandline switches                                                 */
/*======================================================================*/
KNOB<BOOL>   KnobVerbose(KNOB_MODE_WRITEONCE,    "pintool",
    "verbose", "0", "Verbose run");

KNOB<BOOL>   KnobDumpTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "dump_tc", "0", "Dump Translated Code");

KNOB<BOOL>   KnobDoNotCommitTranslatedCode(KNOB_MODE_WRITEONCE,    "pintool",
    "no_tc_commit", "0", "Do not commit translated code");

KNOB<BOOL>   KnobProf(KNOB_MODE_WRITEONCE,    "pintool",
    "prof", "0", "prof - print out loop profie");
	
KNOB<BOOL>   KnobOpt(KNOB_MODE_WRITEONCE,    "pintool",
    "opt", "0", "opt - function inlining and code reordering");

KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE, "pintool",
	"o", "profile.csv", "OutputFile");


/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */


// For XED:
#if defined(TARGET_IA32E)
    xed_state_t dstate = {XED_MACHINE_MODE_LONG_64, XED_ADDRESS_WIDTH_64b};
#else
    xed_state_t dstate = { XED_MACHINE_MODE_LEGACY_32, XED_ADDRESS_WIDTH_32b};
#endif

//For XED: Pass in the proper length: 15 is the max. But if you do not want to
//cross pages, you can pass less than 15 bytes, of course, the
//instruction might not decode if not enough bytes are provided.
const unsigned int max_inst_len = XED_MAX_INSTRUCTION_BYTES;

ADDRINT lowest_sec_addr = 0;
ADDRINT highest_sec_addr = 0;

#define MAX_PROBE_JUMP_INSTR_BYTES  14

int tc_cursor = 0;
int num_of_instr_map_entries = 0;
int max_ins_count = 0;
int max_rtn_count = 0; // total number of routines in the main executable module
int translated_rtn_num = 0;


xed_decoded_inst_t create_uncond_jump_to_next_address(xed_decoded_inst_t orig_xedd, ADDRINT address, ADDRINT jump_address, ADDRINT next_address_new)
{
	char diss_buffer[2048];
	xed_format_context(XED_SYNTAX_INTEL, &orig_xedd, diss_buffer, 2048, address, 0, 0);

	cerr << "orig instr: 0x" << hex << address << " " << diss_buffer << endl;
	cerr << "next_address_new = 0x" << next_address_new << ", next_address_old = 0x" << jump_address << endl;
	cerr << "adding uncond jump to 0x" << jump_address << endl << endl;

	// create a direct uncond jump to the same address:
	xed_decoded_inst_t new_xedd;
	xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
	unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;
	unsigned int new_size = 0;
	xed_int32_t disp = 0;
	xed_encoder_instruction_t  enc_instr;

	xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_JMP, 64,
			xed_relbr(disp, 32));
							
	xed_encoder_request_t enc_req;

	xed_encoder_request_zero_set_mode(&enc_req, &dstate);
	xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
	if (!convert_ok)
	{
		cerr << "conversion to encode request failed" << endl;
		return new_xedd;
	}

	xed_error_enum_t xed_error = xed_encode (&enc_req, enc_buf, max_size, &new_size);
	if (xed_error != XED_ERROR_NONE)
	{
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;				
		return new_xedd;
	}		
	
	xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);

	xed_error = xed_decode(&new_xedd, enc_buf, XED_MAX_INSTRUCTION_BYTES);
	if (xed_error != XED_ERROR_NONE)
	{
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << address << endl;
		return new_xedd;
	}

	uncond_jmps_Map[address].after_which_address = address;
	uncond_jmps_Map[address].jump_address = jump_address;
	uncond_jmps_Map[address].xedd = new_xedd;

	return new_xedd;
}


void revert_branches_and_add_jumps(map<ADDRINT, vector<ADDRINT>> &new_routine_order_Map, map<ADDRINT, xed_decoded_inst_t> &local_instrs_map)
{
	for(const auto& rtn_order : new_routine_order_Map)
	{
        vector<ADDRINT> new_order_vec = rtn_order.second;

		for (size_t i = 0; i < new_order_vec.size() - 1; i++)
		{
			ADDRINT curr_address = new_order_vec[i];
			ADDRINT next_address_new = new_order_vec[i + 1];
			xed_decoded_inst_t xedd = local_instrs_map[curr_address];
			xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
			ADDRINT next_address_old = curr_address + xed_decoded_inst_get_length(&xedd);
	  		ADDRINT orig_target_address = curr_address + xed_decoded_inst_get_length(&xedd) + xed_decoded_inst_get_branch_displacement(&xedd);

			if(category_enum == XED_CATEGORY_UNCOND_BR || category_enum == XED_CATEGORY_CALL)
				continue;

			if(category_enum == XED_CATEGORY_RET)
			{
				cerr << "handling inlined ret with uncond jump" << endl;
				ADDRINT rtn_address = RTN_Address(RTN_FindByAddress(curr_address));

				if(routines_to_commit_Map.find(rtn_address) != routines_to_commit_Map.end())
					continue;
				
				// if it enters here, it is an inlined function so we add jump after ret (for the reordering)
				if(routines_to_translate_Uset.find(rtn_address) == routines_to_translate_Uset.end())
				{
					ADDRINT target_address;
					// loop through calls_to_inline_Map and find which call jumps to this rtn_address
					for(const auto& call : calls_to_inline_Map)
					{
						if(call.second == rtn_address)
						{
							target_address = call.first;
							break;
						}
					}
					xed_decoded_inst_t call_xedd = local_instrs_map[target_address];
					target_address = target_address + xed_decoded_inst_get_length(&call_xedd);
					create_uncond_jump_to_next_address(xedd, curr_address, target_address, 0);
				}
			}
			
			if (category_enum != XED_CATEGORY_COND_BR)
			{
				if(next_address_new != next_address_old)
				{
					create_uncond_jump_to_next_address(xedd, curr_address, next_address_old, next_address_new);
				}
				continue;
			}
			
			if(next_address_new == next_address_old)
				continue;


			if(next_address_new == orig_target_address)
			{
					
				xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(&xedd);

				if (iclass_enum == XED_ICLASS_JRCXZ) // do not revert JRCXZ
					continue;

				xed_iclass_enum_t retverted_iclass = get_reverted_iclass_enum(iclass_enum);
				if (retverted_iclass == XED_ICLASS_INVALID)
					continue;
					
				cerr << "----- reverting instruction -----" << endl;
				cerr << "old iclass = " << xed_iclass_enum_t2str(iclass_enum) << ", new iclass = " << xed_iclass_enum_t2str(retverted_iclass) << endl;
				cerr << "next_address_new = " << next_address_new << ", next_address_old = " << next_address_old << " orig_target_address = " << orig_target_address << endl;
				
				xed_decoded_inst_t new_xedd = xedd;
				// Converts the decoder request to a valid encoder request:
				xed_encoder_request_init_from_decode(&new_xedd);
				// set the reverted opcode;
				xed_encoder_request_set_iclass(&new_xedd, retverted_iclass);

				xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
				unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;
				unsigned int new_size = 0;
			
				xed_error_enum_t xed_error = xed_encode (&new_xedd, enc_buf, max_size, &new_size);
				if (xed_error != XED_ERROR_NONE)
				{
					cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) <<  endl;
					continue;
				}

				xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);
				xed_error_enum_t xed_code = xed_decode(&new_xedd, enc_buf, XED_MAX_INSTRUCTION_BYTES);
				if (xed_code != XED_ERROR_NONE)
				{
					cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << curr_address << endl;
					continue;
				}

				print_original_and_new_instr(xedd, new_xedd, curr_address);

				local_instrs_map[curr_address] = new_xedd;
				reverted_cond_jmps_Map[curr_address] = next_address_new;

				cerr << "----- finished reverting -----" << endl << endl;
			}

			// create a direct uncond jump to the next address if the fall through is not the original jump address
			if(next_address_new != orig_target_address)
			{
				create_uncond_jump_to_next_address(xedd, curr_address, next_address_old, next_address_new);
			}


		}

		// handle the last instruction of the new ordered routine
		ADDRINT curr_address = new_order_vec[new_order_vec.size() - 1];
		xed_decoded_inst_t xedd = local_instrs_map[curr_address];
		xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
		ADDRINT next_address_old = curr_address + xed_decoded_inst_get_length(&xedd);
		cerr << "xed_decoded_inst_get_length(&xedd) is = " << xed_decoded_inst_get_length(&xedd) << endl;
		if(category_enum == XED_CATEGORY_UNCOND_BR || category_enum == XED_CATEGORY_CALL || category_enum == XED_CATEGORY_RET)
			continue;

		create_uncond_jump_to_next_address(xedd, curr_address, next_address_old, 0);
	}
}
/* ============================================================= */
/* Service dump routines                                         */
/* ============================================================= */

/*************************/
/* dump_all_image_instrs */
/*************************/
void dump_all_image_instrs(IMG img)
{
	for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {   
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {		

			// Open the RTN.
            RTN_Open( rtn );

			cerr << RTN_Name(rtn) << ":" << endl;

			for( INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins) )
            {				
	              cerr << "0x" << hex << INS_Address(ins) << ": " << INS_Disassemble(ins) << endl;
			}

			// Close the RTN.
            RTN_Close( rtn );
		}
	}
}


/*************************/
/* dump_instr_from_xedd */
/*************************/
void dump_instr_from_xedd (xed_decoded_inst_t* xedd, ADDRINT address)
{
	// debug print decoded instr:
	char disasm_buf[2048];

    xed_uint64_t runtime_address = static_cast<UINT64>(address);  // set the runtime adddress for disassembly 	

    xed_format_context(XED_SYNTAX_INTEL, xedd, disasm_buf, sizeof(disasm_buf), static_cast<UINT64>(runtime_address), 0, 0);	

    cerr << hex << address << ": " << disasm_buf <<  endl;
}


/************************/
/* dump_instr_from_mem */
/************************/
void dump_instr_from_mem (ADDRINT *address, ADDRINT new_addr)
{
  char disasm_buf[2048];
  xed_decoded_inst_t new_xedd;

  xed_decoded_inst_zero_set_mode(&new_xedd,&dstate); 
   
  xed_error_enum_t xed_code = xed_decode(&new_xedd, reinterpret_cast<UINT8*>(address), max_inst_len);				   

  BOOL xed_ok = (xed_code == XED_ERROR_NONE);
  if (!xed_ok){
	  cerr << "invalid opcode" << endl;
	  return;
  }
 
  xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048, static_cast<UINT64>(new_addr), 0, 0);

  cerr << "0x" << hex << new_addr << ": " << disasm_buf <<  endl;  
 
}


/****************************/
/*  dump_entire_instr_map() */
/****************************/
void dump_entire_instr_map()
{	
	for (int i=0; i < num_of_instr_map_entries; i++) {
		for (int j=0; j < translated_rtn_num; j++) {
			if (translated_rtn[j].instr_map_entry == i) {

				RTN rtn = RTN_FindByAddress(translated_rtn[j].rtn_addr);

				if (rtn == RTN_Invalid()) {
					cerr << "Unknwon"  << ":" << endl;
				} else {
				  cerr << RTN_Name(rtn) << ":" << endl;
				}
			}
		}
		dump_instr_from_mem ((ADDRINT *)instr_map[i].new_ins_addr, instr_map[i].new_ins_addr);		
	}
}


/**************************/
/* dump_instr_map_entry */
/**************************/
void dump_instr_map_entry(int instr_map_entry)
{
	cerr << dec << instr_map_entry << ": ";
	cerr << " orig_ins_addr: " << hex << instr_map[instr_map_entry].orig_ins_addr;
	cerr << " new_ins_addr: " << hex << instr_map[instr_map_entry].new_ins_addr;
	cerr << " orig_targ_addr: " << hex << instr_map[instr_map_entry].orig_targ_addr;

	ADDRINT new_targ_addr;
	if (instr_map[instr_map_entry].targ_map_entry >= 0)
		new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;
	else
		new_targ_addr = instr_map[instr_map_entry].orig_targ_addr;

	cerr << " new_targ_addr: " << hex << new_targ_addr << endl;
	cerr << "    new instr:";
	dump_instr_from_mem((ADDRINT *)instr_map[instr_map_entry].encoded_ins, instr_map[instr_map_entry].new_ins_addr);
}


/*************/
/* dump_tc() */
/*************/
void dump_tc()
{
  char disasm_buf[2048];
  xed_decoded_inst_t new_xedd;
  ADDRINT address = (ADDRINT)&tc[0];
  unsigned int size = 0;

  while (address < (ADDRINT)&tc[tc_cursor]) {

      address += size;

	  xed_decoded_inst_zero_set_mode(&new_xedd,&dstate); 
   
	  xed_error_enum_t xed_code = xed_decode(&new_xedd, reinterpret_cast<UINT8*>(address), max_inst_len);				   

	  BOOL xed_ok = (xed_code == XED_ERROR_NONE);
	  if (!xed_ok){
		  cerr << "invalid opcode" << endl;
		  return;
	  }
 
	  xed_format_context(XED_SYNTAX_INTEL, &new_xedd, disasm_buf, 2048, static_cast<UINT64>(address), 0, 0);

	  cerr << "0x" << hex << address << ": " << disasm_buf <<  endl;

	  size = xed_decoded_inst_get_length (&new_xedd);	
  }
}


/* ============================================================= */
/* Translation routines                                         */
/* ============================================================= */

/*************************/
/* add_new_uncond_jump_instr_entry() */
/*************************/
int add_new_uncond_jump_instr_entry(xed_decoded_inst_t *xedd, ADDRINT pc, unsigned int size)
{

	//debug print of orig instruction:
	if (KnobVerbose)
	{
		char disasm_buf[2048];
		xed_format_context(XED_SYNTAX_INTEL, xedd, disasm_buf, 2048, static_cast<UINT64>(pc), 0, 0);               
		cerr << "	old jump instr : 0x" << hex << pc << ": " << disasm_buf  <<  endl;
		cerr << "	new target address is 0x" << hex << uncond_jmps_Map[pc].jump_address << endl;
	}

	if (xed_decoded_inst_get_length (xedd) != size)
	{
		cerr << "Invalid instruction decoding" << endl;
		return -1;
	}

	// Converts the decoder request to a valid encoder request:
	xed_encoder_request_init_from_decode (xedd);

    unsigned int new_size = 0;
	
	xed_error_enum_t xed_error = xed_encode (xedd, reinterpret_cast<UINT8*>(instr_map[num_of_instr_map_entries].encoded_ins), max_inst_len , &new_size);
	if (xed_error != XED_ERROR_NONE) 
	{
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;		
		return -1;
	}	

	ADDRINT orig_targ_addr = uncond_jmps_Map[pc].jump_address;

	// add a new entry in the instr_map:
	instr_map[num_of_instr_map_entries].new_ins_addr = (ADDRINT)&tc[tc_cursor];  // set an initial estimated addr in tc
	instr_map[num_of_instr_map_entries].orig_targ_addr = orig_targ_addr; 
    instr_map[num_of_instr_map_entries].hasNewTargAddr = false;
	instr_map[num_of_instr_map_entries].targ_map_entry = -1;
	instr_map[num_of_instr_map_entries].size = new_size;	
    instr_map[num_of_instr_map_entries].category_enum = xed_decoded_inst_get_category(xedd);
	num_of_instr_map_entries++;

	// update expected size of tc:
	tc_cursor += new_size;    	     

	if (num_of_instr_map_entries >= max_ins_count)
	{
		cerr << "out of memory for map_instr" << endl;
		return -1;
	}
	
    // debug print new encoded instr:
	if (KnobVerbose)
	{
		cerr << "    new uncond jump instr : ";
		dump_instr_from_mem((ADDRINT *)instr_map[num_of_instr_map_entries-1].encoded_ins, instr_map[num_of_instr_map_entries-1].new_ins_addr);
	}

	return new_size;
}


/*************************/
/* add_new_instr_entry() */
/*************************/
int add_new_instr_entry(xed_decoded_inst_t *xedd, ADDRINT pc, unsigned int size)
{

	// copy orig instr to instr map:
    ADDRINT orig_targ_addr = 0;

	if (xed_decoded_inst_get_length (xedd) != size)
	{
		cerr << "Invalid instruction decoding" << endl;
		return -1;
	}

    xed_uint_t disp_byts = xed_decoded_inst_get_branch_displacement_width(xedd);
	xed_int32_t disp;

 	// there is a branch offset.
    if (disp_byts > 0)
	{
      disp = xed_decoded_inst_get_branch_displacement(xedd);
	  orig_targ_addr = pc + xed_decoded_inst_get_length (xedd) + disp;	
	}

	// Converts the decoder request to a valid encoder request:
	xed_encoder_request_init_from_decode (xedd);

    unsigned int new_size = 0;
	
	xed_error_enum_t xed_error = xed_encode (xedd, reinterpret_cast<UINT8*>(instr_map[num_of_instr_map_entries].encoded_ins), max_inst_len , &new_size);
	if (xed_error != XED_ERROR_NONE) 
	{
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;		
		return -1;
	}	
	
	// if this is a reverted cond branch, we need to update the orig target address
	if (reverted_cond_jmps_Map.find(pc) != reverted_cond_jmps_Map.end())
	{
		orig_targ_addr = reverted_cond_jmps_Map[pc];
	}

	// add a new entry in the instr_map:
	instr_map[num_of_instr_map_entries].orig_ins_addr = pc;
	instr_map[num_of_instr_map_entries].new_ins_addr = (ADDRINT)&tc[tc_cursor];  // set an initial estimated addr in tc
	instr_map[num_of_instr_map_entries].orig_targ_addr = orig_targ_addr; 
    instr_map[num_of_instr_map_entries].hasNewTargAddr = false;
	instr_map[num_of_instr_map_entries].targ_map_entry = -1;
	instr_map[num_of_instr_map_entries].size = new_size;	
    instr_map[num_of_instr_map_entries].category_enum = xed_decoded_inst_get_category(xedd);

	num_of_instr_map_entries++;

	// update expected size of tc:
	tc_cursor += new_size;    	     

	if (num_of_instr_map_entries >= max_ins_count)
	{
		cerr << "out of memory for map_instr" << endl;
		return -1;
	}
	
    // debug print new encoded instr:
	if (KnobVerbose)
	{
		cerr << "    new instr:";
		dump_instr_from_mem((ADDRINT *)instr_map[num_of_instr_map_entries-1].encoded_ins, instr_map[num_of_instr_map_entries-1].new_ins_addr);
	}

	return new_size;
}


/*************************************************/
/* chain_all_direct_br_and_call_target_entries() */
/*************************************************/
int chain_all_direct_br_and_call_target_entries()
{
	for (int i = 0; i < num_of_instr_map_entries; i++) {			    

		if (instr_map[i].orig_targ_addr == 0)
			continue;

		if (instr_map[i].hasNewTargAddr)
			continue;

        for (int j = 0; j < num_of_instr_map_entries; j++) {

            if (j == i)
			   continue;

            if (instr_map[j].orig_ins_addr == instr_map[i].orig_targ_addr)
			{
                instr_map[i].hasNewTargAddr = true; 
	            instr_map[i].targ_map_entry = j;
                break;
			}
		}
	}
   
	return 0;
}


/**************************/
/* fix_rip_displacement() */
/**************************/
int fix_rip_displacement(int instr_map_entry) 
{
	//debug print:
	//dump_instr_map_entry(instr_map_entry);

	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}

	unsigned int memops = xed_decoded_inst_number_of_memory_operands(&xedd);

	if (instr_map[instr_map_entry].orig_targ_addr != 0)  // a direct jmp or call instruction.
		return 0;

	//cerr << "Memory Operands" << endl;
	bool isRipBase = false;
	xed_reg_enum_t base_reg = XED_REG_INVALID;
	xed_int64_t disp = 0;
	for(unsigned int i=0; i < memops ; i++)   {

		base_reg = xed_decoded_inst_get_base_reg(&xedd,i);
		disp = xed_decoded_inst_get_memory_displacement(&xedd,i);

		if (base_reg == XED_REG_RIP) {
			isRipBase = true;
			break;
		}
	}
	if (!isRipBase)
		return 0;

			
	//xed_uint_t disp_byts = xed_decoded_inst_get_memory_displacement_width(xedd,i); // how many byts in disp ( disp length in byts - for example FFFFFFFF = 4
	xed_int64_t new_disp = 0;
	xed_uint_t new_disp_byts = 4;   // set maximal num of byts for now.

	unsigned int orig_size = xed_decoded_inst_get_length (&xedd);

	// modify rip displacement. use direct addressing mode:	
	new_disp = instr_map[instr_map_entry].orig_ins_addr + disp + orig_size; // xed_decoded_inst_get_length (&xedd_orig);
	xed_encoder_request_set_base0 (&xedd, XED_REG_INVALID);

	//Set the memory displacement using a bit length 
	xed_encoder_request_set_memory_displacement (&xedd, new_disp, new_disp_byts);

	unsigned int size = XED_MAX_INSTRUCTION_BYTES;
	unsigned int new_size = 0;
			
	// Converts the decoder request to a valid encoder request:
	xed_encoder_request_init_from_decode (&xedd);
	
	xed_error_enum_t xed_error = xed_encode (&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), size , &new_size); // &instr_map[i].size
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
		dump_instr_map_entry(instr_map_entry); 
		return -1;
	}				

	if (KnobVerbose) {
		dump_instr_map_entry(instr_map_entry);
	}

	return new_size;
}


/************************************/
/* fix_direct_br_call_to_orig_addr */
/************************************/
int fix_direct_br_call_to_orig_addr(int instr_map_entry)
{

	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}
	
	xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
	
	if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_UNCOND_BR) {

		cerr << "ERROR: Invalid direct jump from translated code to original code in routine: " 
			  << RTN_Name(RTN_FindByAddress(instr_map[instr_map_entry].orig_ins_addr)) << endl;
		dump_instr_map_entry(instr_map_entry);
		return -1;
	}

	// check for cases of direct jumps/calls back to the orginal target address:
	if (instr_map[instr_map_entry].targ_map_entry >= 0) {
		cerr << "ERROR: Invalid jump or call instruction" << endl;
		return -1;
	}

	unsigned int ilen = XED_MAX_INSTRUCTION_BYTES;
	unsigned int olen = 0;
				

	xed_encoder_instruction_t  enc_instr;

	ADDRINT new_disp = (ADDRINT)&instr_map[instr_map_entry].orig_targ_addr - 
		               instr_map[instr_map_entry].new_ins_addr - 
					   xed_decoded_inst_get_length (&xedd);

	if (category_enum == XED_CATEGORY_CALL)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_CALL_NEAR, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

	if (category_enum == XED_CATEGORY_UNCOND_BR)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_JMP, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));


	xed_encoder_request_t enc_req;

	xed_encoder_request_zero_set_mode(&enc_req, &dstate);
	xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
	if (!convert_ok) {
		cerr << "conversion to encode request failed" << endl;
		return -1;
	}
   

	xed_error_enum_t xed_error = xed_encode(&enc_req, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), ilen, &olen);
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
	    dump_instr_map_entry(instr_map_entry); 
        return -1;
    }

	// handle the case where the original instr size is different from new encoded instr:
	if (olen != xed_decoded_inst_get_length (&xedd)) {
		
		new_disp = (ADDRINT)&instr_map[instr_map_entry].orig_targ_addr - 
	               instr_map[instr_map_entry].new_ins_addr - olen;

		if (category_enum == XED_CATEGORY_CALL)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_CALL_NEAR, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));

		if (category_enum == XED_CATEGORY_UNCOND_BR)
			xed_inst1(&enc_instr, dstate, 
			XED_ICLASS_JMP, 64,
			xed_mem_bd (XED_REG_RIP, xed_disp(new_disp, 32), 64));


		xed_encoder_request_zero_set_mode(&enc_req, &dstate);
		xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
		if (!convert_ok) {
			cerr << "conversion to encode request failed" << endl;
			return -1;
		}

		xed_error = xed_encode (&enc_req, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), ilen , &olen);
		if (xed_error != XED_ERROR_NONE) {
			cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
			dump_instr_map_entry(instr_map_entry);
			return -1;
		}		
	}

	// debug prints:
	if (KnobVerbose) {
		dump_instr_map_entry(instr_map_entry); 
	}
		
	instr_map[instr_map_entry].hasNewTargAddr = true;
	return olen;	
}


/***********************************/
/* fix_direct_br_call_displacement */
/***********************************/
int fix_direct_br_call_displacement(int instr_map_entry) 
{					

	xed_decoded_inst_t xedd;
	xed_decoded_inst_zero_set_mode(&xedd,&dstate); 
				   
	xed_error_enum_t xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), max_inst_len);
	if (xed_code != XED_ERROR_NONE) {
		cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << instr_map[instr_map_entry].new_ins_addr << endl;
		return -1;
	}

	xed_int32_t  new_disp = 0;	
	unsigned int size = XED_MAX_INSTRUCTION_BYTES;
	unsigned int new_size = 0;


	xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
	
	if (category_enum != XED_CATEGORY_CALL && category_enum != XED_CATEGORY_COND_BR && category_enum != XED_CATEGORY_UNCOND_BR) {
		cerr << "ERROR: unrecognized branch displacement" << endl;
		return -1;
	}

	// fix branches/calls to original targ addresses:
	if (instr_map[instr_map_entry].targ_map_entry < 0) {
	   int rc = fix_direct_br_call_to_orig_addr(instr_map_entry);
	   return rc;
	}

	ADDRINT new_targ_addr;		
	new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;
		
	new_disp = (new_targ_addr - instr_map[instr_map_entry].new_ins_addr) - instr_map[instr_map_entry].size; // orig_size;

	xed_uint_t   new_disp_byts = 4; // num_of_bytes(new_disp);  ???

	// the max displacement size of loop instructions is 1 byte:
	xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(&xedd);
	if (iclass_enum == XED_ICLASS_LOOP ||  iclass_enum == XED_ICLASS_LOOPE || iclass_enum == XED_ICLASS_LOOPNE) {
	  new_disp_byts = 1;
	}

	// the max displacement size of jecxz instructions is ???:
	xed_iform_enum_t iform_enum = xed_decoded_inst_get_iform_enum (&xedd);
	if (iform_enum == XED_IFORM_JRCXZ_RELBRb){
	  new_disp_byts = 1;
	}

	// Converts the decoder request to a valid encoder request:
	xed_encoder_request_init_from_decode (&xedd);

	//Set the branch displacement:
	xed_encoder_request_set_branch_displacement (&xedd, new_disp, new_disp_byts);

	xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
	unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;
    
	xed_error_enum_t xed_error = xed_encode (&xedd, enc_buf, max_size , &new_size);
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) <<  endl;
		char buf[2048];		
		xed_format_context(XED_SYNTAX_INTEL, &xedd, buf, 2048, static_cast<UINT64>(instr_map[instr_map_entry].orig_ins_addr), 0, 0);
	    cerr << " instr: " << "0x" << hex << instr_map[instr_map_entry].orig_ins_addr << " : " << buf <<  endl;
  		return -1;
	}		

	new_targ_addr = instr_map[instr_map[instr_map_entry].targ_map_entry].new_ins_addr;

	new_disp = new_targ_addr - (instr_map[instr_map_entry].new_ins_addr + new_size);  // this is the correct displacemnet.

	//Set the branch displacement:
	xed_encoder_request_set_branch_displacement (&xedd, new_disp, new_disp_byts);
	
	xed_error = xed_encode (&xedd, reinterpret_cast<UINT8*>(instr_map[instr_map_entry].encoded_ins), size , &new_size); // &instr_map[i].size
	if (xed_error != XED_ERROR_NONE) {
		cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;
		dump_instr_map_entry(instr_map_entry);
		return -1;
	}				

	//debug print of new instruction in tc:
	if (KnobVerbose) {
		dump_instr_map_entry(instr_map_entry);
	}

	return new_size;
}				


/************************************/
/* fix_instructions_displacements() */
/************************************/
int fix_instructions_displacements()
{
   // fix displacemnets of direct branch or call instructions:

    int size_diff = 0;	

	do {
		
		size_diff = 0;

		if (KnobVerbose)
		{
			cerr << "----------------------------------------------------------" << endl;
			cerr << "starting a pass of fixing instructions displacements: " << endl;
		}

		for (int i=0; i < num_of_instr_map_entries; i++) {

			instr_map[i].new_ins_addr += size_diff;
				   
			int new_size = 0;

			// fix rip displacement:			
			new_size = fix_rip_displacement(i);
			if (new_size < 0)
				return -1;

			if (new_size > 0) { // this was a rip-based instruction which was fixed.

				if (instr_map[i].size != (unsigned int)new_size) {
				   size_diff += (new_size - instr_map[i].size); 					
				   instr_map[i].size = (unsigned int)new_size;								
				}

				continue;   
			}

			// check if it is a direct branch or a direct call instr:
			if (instr_map[i].orig_targ_addr == 0) {
				continue;  // not a direct branch or a direct call instr.
			}


			// fix instr displacement:			
			new_size = fix_direct_br_call_displacement(i);
			if (new_size < 0)
				return -1;

			if (instr_map[i].size != (unsigned int)new_size) {
			   size_diff += (new_size - instr_map[i].size);
			   instr_map[i].size = (unsigned int)new_size;
			}

		}  // end int i=0; i ..

	} while (size_diff != 0);

   return 0;
 }




/*****************************************/
/* find_candidate_rtns_for_translation() */
/*****************************************/
int find_candidate_rtns_for_translation(IMG img)
{
	map<ADDRINT, xed_decoded_inst_t> local_instrs_map;
	local_instrs_map.clear();

	// go over routines and check if they are candidates for translation and mark them for translation:

	for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
	{   
		if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
		continue;

		for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
		{    

			if (rtn == RTN_Invalid())
			{
			cerr << "Warning: invalid routine " << RTN_Name(rtn) << endl;
			continue;
			}

			if(routines_to_translate_Uset.find(RTN_Address(rtn)) == routines_to_translate_Uset.end())
			{
				continue;
			}

			// if we want to translate specific functions
			// if(RTN_Address(rtn) != 0x401278 && RTN_Address(rtn) != 0x401090)
			// {
			// 	continue;
			// }

			translated_rtn[translated_rtn_num].rtn_addr = RTN_Address(rtn);            
			translated_rtn[translated_rtn_num].rtn_size = RTN_Size(rtn);

			if(routines_to_commit_Map.find(RTN_Address(rtn)) != routines_to_commit_Map.end())
			{
				routines_to_commit_Map[RTN_Address(rtn)] = translated_rtn_num;
			}

			// Open the RTN.
			RTN_Open( rtn ); 

			for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins))
			{
				ADDRINT addr = INS_Address(ins);

				xed_decoded_inst_t xedd;
				xed_error_enum_t xed_code;                            

				xed_decoded_inst_zero_set_mode(&xedd,&dstate); 

				xed_code = xed_decode(&xedd, reinterpret_cast<UINT8*>(addr), max_inst_len);
				if (xed_code != XED_ERROR_NONE)
				{
					cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << addr << endl;
					translated_rtn[translated_rtn_num].instr_map_entry = -1;
					break;
				}

				// Save xed and addr into a map to be used later.
				local_instrs_map[addr] = xedd;

			} // end for INS...


			// debug print of routine name:
			if (KnobVerbose)
				cerr << "saved instructions from rtn num [" << dec << translated_rtn_num << "] named : [" <<  RTN_Name(rtn) << "]" << endl;

			// Close the RTN.
			RTN_Close( rtn );

			translated_rtn_num++;

		} // end for RTN..
	} // end for SEC...

	cerr << "-------------------- Finished saving instruction into local_instrs_map ---------------------" << endl;

	// Create a set and a vector of instructions for each routine
	map<ADDRINT, set<ADDRINT>> rtn_instrs_set_map;

	for (const auto& local_inst : local_instrs_map)
	{
		ADDRINT caller_ins_addr = local_inst.first;
		ADDRINT rtn_address = RTN_Address(RTN_FindByAddress(caller_ins_addr));

		rtn_instrs_set_map[rtn_address].insert(caller_ins_addr);\
	}

	map<ADDRINT, set<ADDRINT>> rtn_instrs_set_map_copy = rtn_instrs_set_map;


	// For each routine, create a vector of instructions in a new order
	map<ADDRINT, vector<ADDRINT>> new_routine_order_Map;
	for (const auto& rtn_instrs_set : rtn_instrs_set_map)
	{
		ADDRINT rtn_address = rtn_instrs_set.first;
		set<ADDRINT> instrs_set = rtn_instrs_set.second;

		vector<ADDRINT> &new_order_vec = new_routine_order_Map[rtn_address];
		cerr << "@@@@@@@@@@" << endl;	
		cerr << "new order of instructions for rtn [" << RTN_Name(RTN_FindByAddress(rtn_address)) << "]:" << endl;
		// Loop through reorder_Map[rtn_address] and add the instructions in the new order
		for(ADDRINT bbl_Address : reorder_Map[rtn_address])
		{

			// get iterator to bbl_Address in rtn_instrs_set_map[rtn_address]
			set<ADDRINT>::iterator it = instrs_set.find(bbl_Address);
			while(it != instrs_set.end())
			{	
				// check if the ins is not in the new_order_vec
				if(find(new_order_vec.begin(), new_order_vec.end(), *it) != new_order_vec.end())
				{
					cerr << "-- tail" << endl;
					break;
				}

				// insert the ins to the new_order_vec
				ADDRINT ins_address = *it;
				new_order_vec.push_back(ins_address);
				rtn_instrs_set_map_copy[rtn_address].erase(ins_address);

				xed_decoded_inst_t xedd = local_instrs_map[ins_address];
				char disasm_buf[2048];
				xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(ins_address), 0, 0);               
				cerr << "0x" << hex << ins_address << ": " << disasm_buf  <<  endl; 

				// get the xedd ins from local_instrs_map and check if ins is a tail instruction
				xed_category_enum_t category_enum = xed_decoded_inst_get_category(&xedd);
				if (category_enum == XED_CATEGORY_CALL 		|| category_enum == XED_CATEGORY_RET || 
					category_enum == XED_CATEGORY_UNCOND_BR || category_enum == XED_CATEGORY_COND_BR)
				{
					cerr << "-- tail" << endl;
					break;
				}

				it++;
			}
		}
		
		cerr << "inserting leftover instructions..." << endl;

		// Loop through rtn_instrs_set_map[rtn_address] and add the instructions that are not in in the new order
		for(ADDRINT ins_address : rtn_instrs_set_map_copy[rtn_address])
		{
			new_order_vec.push_back(ins_address);
			xed_decoded_inst_t xedd = local_instrs_map[ins_address];
			char disasm_buf[2048];
			xed_format_context(XED_SYNTAX_INTEL, &xedd, disasm_buf, 2048, static_cast<UINT64>(ins_address), 0, 0);               
			cerr << "0x" << hex << ins_address << ": " << disasm_buf  <<  endl; 
		}
		cerr << "finished" << endl;
	}

	// Go over new_routine_order_Map and revert conditional branches of bbls that were reordered
	// also add uncond jumps after if needed
	revert_branches_and_add_jumps(new_routine_order_Map, local_instrs_map);

	int curr_rtn_num = 0;

	// // Go over the local_instrs_map map and add each instruction to the instr_map
	// for (const auto& local_inst : local_instrs_map)
	// {
	
	// Go over the new_routine_order_Map and for each rtn translate the instructions
	for (auto& rtn_ins_vec : new_routine_order_Map)
	{
		vector<ADDRINT> &new_order_vec = rtn_ins_vec.second;

		// Go over the new_order_vec
		for(size_t i = 0 ; i < new_order_vec.size() ; i++)
		{
			
			// ADDRINT ins_addr = local_inst.first;
			// xed_decoded_inst_t caller_xedd = local_inst.second;

			ADDRINT ins_addr = new_order_vec[i];
			xed_decoded_inst_t caller_xedd = local_instrs_map[ins_addr];

			// Check if we want to add this routine to the global instr_map
			ADDRINT rtn_address = RTN_Address(RTN_FindByAddress(ins_addr));
			if(routines_to_commit_Map.find(rtn_address) == routines_to_commit_Map.end())
			{
				continue;
			}

			curr_rtn_num = routines_to_commit_Map[rtn_address];

			// Check if we are at a routine header:
			if (translated_rtn[curr_rtn_num].rtn_addr == ins_addr)
			{
				// debug print of routine name:
				if (KnobVerbose)
				{
					cerr << endl << "$$$$$$$$$$ New Routine $$$$$$$$$$" << endl;
					cerr << "translating rtn num [" << dec << curr_rtn_num << "] named : [" <<  RTN_FindNameByAddress(ins_addr) << "]" << endl << endl;
				}

				translated_rtn[curr_rtn_num].instr_map_entry = num_of_instr_map_entries;
				translated_rtn[curr_rtn_num].isSafeForReplacedProbe = true;
			}

			//debug print of orig instruction:
			if (KnobVerbose)
			{
				char disasm_buf[2048];
				xed_format_context(XED_SYNTAX_INTEL, &caller_xedd, disasm_buf, 2048, static_cast<UINT64>(ins_addr), 0, 0);               
				cerr << "old instr : 0x" << hex << ins_addr << ": " << disasm_buf  <<  endl; 
			}

			bool ins_good_for_inline = calls_to_inline_Map.find(ins_addr) != calls_to_inline_Map.end();

			if(! ins_good_for_inline) // not an inline call
			{
				// Add instr into global instr_map:
				int rc = add_new_instr_entry(&caller_xedd, ins_addr, xed_decoded_inst_get_length(&caller_xedd));
				if (rc < 0)
				{
					cerr << "ERROR: failed during instructon translation." << endl;
					translated_rtn[curr_rtn_num].instr_map_entry = -1;
					break;
				}

				// Check if we need to add uncond jump after this instruction
				if(uncond_jmps_Map.find(ins_addr) != uncond_jmps_Map.end())
				{
					xed_decoded_inst_t jmp_xedd = uncond_jmps_Map[ins_addr].xedd;
					int rc = add_new_uncond_jump_instr_entry(&jmp_xedd, ins_addr, xed_decoded_inst_get_length(&jmp_xedd));
					if (rc < 0)
					{
						cerr << "ERROR: failed during instructon translation." << endl;
						translated_rtn[curr_rtn_num].instr_map_entry = -1;
						break;
					}
				}
			}

			if(ins_good_for_inline) // inline this call
			{
				cerr << "	not translating this call instruction"<<  endl; 

				xed_int64_t disp = xed_decoded_inst_get_branch_displacement(&caller_xedd);
				ADDRINT target_addr = ins_addr + xed_decoded_inst_get_length (&caller_xedd) + disp; 
				RTN rtn = RTN_FindByAddress(target_addr);
				RTN_Open(rtn);
				cerr << "-------------------- Starting Inline In Routine " << RTN_FindNameByAddress(ins_addr) << " ---------------------" << endl;

				// /*

				cerr << "Going through reordered inlined function" << endl;

				vector<ADDRINT> &new_order_vec = new_routine_order_Map[target_addr];

				// loop through new_order_vec and add the instructions in the new order
				for(ADDRINT callee_ins_addr : new_order_vec)
				{
					xed_decoded_inst_t callee_xedd = local_instrs_map[callee_ins_addr];

					//debug print of orig instruction:
					if (KnobVerbose)
					{
						char disasm_buf[2048];
						xed_format_context(XED_SYNTAX_INTEL, &callee_xedd, disasm_buf, 2048, static_cast<UINT64>(callee_ins_addr), 0, 0);
						cerr << "old instr : 0x" << hex << callee_ins_addr << ": " << disasm_buf  <<  endl; 
					}

					xed_category_enum_t category_enum = xed_decoded_inst_get_category(&callee_xedd);
					if (category_enum == XED_CATEGORY_RET)
					{
						cerr << "	not translating this ret, added uncond jump instead"<< endl << endl;

						// Check if we need to add uncond jump after this ret
						if(uncond_jmps_Map.find(callee_ins_addr) != uncond_jmps_Map.end())
						{
							xed_decoded_inst_t jmp_xedd = uncond_jmps_Map[callee_ins_addr].xedd;
							int rc = add_new_uncond_jump_instr_entry(&jmp_xedd, callee_ins_addr, xed_decoded_inst_get_length(&jmp_xedd));
							if (rc < 0)
							{
								cerr << "ERROR: failed during instructon translation." << endl;
								translated_rtn[curr_rtn_num].instr_map_entry = -1;
								break;
							}
						}

						continue;
					}
					// Add instr into global instr_map:
					int rc = add_new_instr_entry(&callee_xedd, callee_ins_addr, xed_decoded_inst_get_length(&callee_xedd));
					if (rc < 0)
					{
						cerr << "ERROR: failed during instructon translation." << endl;
						translated_rtn[curr_rtn_num].instr_map_entry = -1;
						break;
					}

					// Check if we need to add uncond jump after this instruction
					if(uncond_jmps_Map.find(callee_ins_addr) != uncond_jmps_Map.end())
					{
						xed_decoded_inst_t jmp_xedd = uncond_jmps_Map[callee_ins_addr].xedd;
						int rc = add_new_uncond_jump_instr_entry(&jmp_xedd, callee_ins_addr, xed_decoded_inst_get_length(&jmp_xedd));
						if (rc < 0)
						{
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[curr_rtn_num].instr_map_entry = -1;
							break;
						}
					}
				}
				// */
				
				/*
				for (INS ins = RTN_InsHead(rtn); INS_Valid(ins); ins = INS_Next(ins))
				{
					ADDRINT callee_ins_addr = INS_Address(ins);
					xed_decoded_inst_t callee_xedd = local_instrs_map[callee_ins_addr];

					//debug print of orig instruction:
					if (KnobVerbose)
					{
						char disasm_buf[2048];
						xed_format_context(XED_SYNTAX_INTEL, &callee_xedd, disasm_buf, 2048, static_cast<UINT64>(callee_ins_addr), 0, 0);
						cerr << "old instr : 0x" << hex << callee_ins_addr << ": " << disasm_buf  <<  endl; 
					}

					xed_category_enum_t category_enum = xed_decoded_inst_get_category(&callee_xedd);
					if (category_enum == XED_CATEGORY_RET)
					{
						cerr << "	not translating this ret"<<  endl; 
						continue;
					}
					// Add instr into global instr_map:
					int rc = add_new_instr_entry(&callee_xedd, callee_ins_addr, xed_decoded_inst_get_length(&callee_xedd));
					if (rc < 0)
					{
						cerr << "ERROR: failed during instructon translation." << endl;
						translated_rtn[curr_rtn_num].instr_map_entry = -1;
						break;
					}

					// Check if we need to add uncond jump after this instruction
					if(uncond_jmps_Map.find(callee_ins_addr) != uncond_jmps_Map.end())
					{
						xed_decoded_inst_t jmp_xedd = uncond_jmps_Map[callee_ins_addr].xedd;
						int rc = add_new_uncond_jump_instr_entry(&jmp_xedd, callee_ins_addr, xed_decoded_inst_get_length(&jmp_xedd));
						if (rc < 0)
						{
							cerr << "ERROR: failed during instructon translation." << endl;
							translated_rtn[curr_rtn_num].instr_map_entry = -1;
							break;
						}
					}
				}
				*/
				cerr << "-------------------- Finishing Inline In Routine " << RTN_FindNameByAddress(ins_addr)  << "---------------------" << endl;
				RTN_Close(rtn);
			}

		} // end for local_instrs_map
	}

	return 0;
}


/***************************/
/* int copy_instrs_to_tc() */
/***************************/
int copy_instrs_to_tc()
{
	int cursor = 0;

	for (int i=0; i < num_of_instr_map_entries; i++) {

	  if ((ADDRINT)&tc[cursor] != instr_map[i].new_ins_addr) {
		  cerr << "ERROR: Non-matching instruction addresses: " << hex << (ADDRINT)&tc[cursor] << " vs. " << instr_map[i].new_ins_addr << endl;
	      return -1;
	  }	  

	  memcpy(&tc[cursor], &instr_map[i].encoded_ins, instr_map[i].size);

	  cursor += instr_map[i].size;
	}

	return 0;
}


/*************************************/
/* void commit_translated_routines() */
/*************************************/
inline void commit_translated_routines() 
{
	// Commit the translated functions: 
	// Go over the candidate functions and replace the original ones by their new successfully translated ones:

	for (int i=0; i < translated_rtn_num; i++) {

		//replace function by new function in tc
	
		if (translated_rtn[i].instr_map_entry >= 0) {
				    
			if (translated_rtn[i].rtn_size > MAX_PROBE_JUMP_INSTR_BYTES && translated_rtn[i].isSafeForReplacedProbe) {						

				RTN rtn = RTN_FindByAddress(translated_rtn[i].rtn_addr);

				//debug print:				
				if (rtn == RTN_Invalid()) {
					cerr << "committing rtN: Unknown";
				} else {
					cerr << "committing rtN: " << RTN_Name(rtn);
				}
				cerr << " from: 0x" << hex << RTN_Address(rtn) << " to: 0x" << hex << instr_map[translated_rtn[i].instr_map_entry].new_ins_addr << endl;

						
				if (RTN_IsSafeForProbedReplacement(rtn)) {

					AFUNPTR origFptr = RTN_ReplaceProbed(rtn,  (AFUNPTR)instr_map[translated_rtn[i].instr_map_entry].new_ins_addr);							

					if (origFptr == NULL) {
						cerr << "RTN_ReplaceProbed failed.";
					} else {
						cerr << "RTN_ReplaceProbed succeeded. ";
					}
					cerr << " orig routine addr: 0x" << hex << translated_rtn[i].rtn_addr
							<< " replacement routine addr: 0x" << hex << instr_map[translated_rtn[i].instr_map_entry].new_ins_addr << endl;	

					dump_instr_from_mem ((ADDRINT *)translated_rtn[i].rtn_addr, translated_rtn[i].rtn_addr);

					cerr << endl;										
				}												
			}
		}
	}
}


/****************************/
/* allocate_and_init_memory */
/****************************/ 
int allocate_and_init_memory(IMG img) 
{
	// Step 1: Calculate size of executable sections and allocate required memory:
	//
	for (SEC sec = IMG_SecHead(img); SEC_Valid(sec); sec = SEC_Next(sec))
    {   
		if (!SEC_IsExecutable(sec) || SEC_IsWriteable(sec) || !SEC_Address(sec))
			continue;


		if (!lowest_sec_addr || lowest_sec_addr > SEC_Address(sec))
			lowest_sec_addr = SEC_Address(sec);

		if (highest_sec_addr < SEC_Address(sec) + SEC_Size(sec))
			highest_sec_addr = SEC_Address(sec) + SEC_Size(sec);

		// need to avouid using RTN_Open as it is expensive...
        for (RTN rtn = SEC_RtnHead(sec); RTN_Valid(rtn); rtn = RTN_Next(rtn))
        {		

			if (rtn == RTN_Invalid())
				continue;

			max_ins_count += RTN_NumIns(rtn);
			max_rtn_count++;
		}
	}

	max_ins_count *= 4; // estimating that the num of instrs of the inlined functions will not exceed the total number of the entire code.
	
	// Step 2: Allocate memory for the instr map needed to fix all branch targets in translated routines:
	// no need because we are using a new map
	instr_map = (instr_map_t *)calloc(max_ins_count, sizeof(instr_map_t));
	if (instr_map == NULL)
	{
		perror("calloc");
		return -1;
	}


	// Step 3: Allocate memory for the array of candidate routines containing inlineable function calls:
	// Need to estimate size of inlined routines.. ???
	translated_rtn = (translated_rtn_t *)calloc(max_rtn_count, sizeof(translated_rtn_t));
	if (translated_rtn == NULL) {
		perror("calloc");
		return -1;
	}


	// Step 4: get a page size in the system:
	int pagesize = sysconf(_SC_PAGE_SIZE);
    if (pagesize == -1) {
      perror("sysconf");
	  return -1;
	}

	ADDRINT text_size = (highest_sec_addr - lowest_sec_addr) * 2 + pagesize * 4;

    int tclen = 2 * text_size + pagesize * 4;   // need a better estimate???

	// Step 5: Allocate the needed tc with RW+EXEC permissions and is not located in an address that is more than 32bits afar:		
	char * addr = (char *) mmap(NULL, tclen, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
	if ((ADDRINT) addr == 0xffffffffffffffff) {
		cerr << "failed to allocate tc" << endl;
        return -1;
	}
	
	tc = (char *)addr;
	return 0;
}



/* ============================================ */
/* Main translation routine                     */
/* ============================================ */
VOID ImageLoad(IMG img, VOID *v)
{
	// debug print of all image's instructions
	// dump_all_image_instrs(img);


    // Step 0: Check the image and the CPU:
	if (!IMG_IsMainExecutable(img))
		return;

	int rc = 0;

	// Step 1: Check size of executable sections and allocate required memory:	
	rc = allocate_and_init_memory(img);
	if (rc < 0)
		return;

	cout << "after memory allocation \n" << endl;

	
	// Step 2: go over all routines and identify candidate routines and copy their code into the instr map IR:
	rc = find_candidate_rtns_for_translation(img);
	if (rc < 0)
		return;

	cout << "after identifying candidate routines \n" << endl;	 
	
	// Step 3: Chaining - calculate direct branch and call instructions to point to corresponding target instr entries:
	rc = chain_all_direct_br_and_call_target_entries();
	if (rc < 0 )
		return;
	
	cout << "after calculate direct br targets \n" << endl;

	// Step 4: fix rip-based, direct branch and direct call displacements:
	rc = fix_instructions_displacements();
	if (rc < 0 )
		return;
	
	cout << "after fix instructions displacements \n" << endl;


	// Step 5: write translated routines to new tc:
	rc = copy_instrs_to_tc();
	if (rc < 0 )
		return;

	cout << "after write all new instructions to memory tc\n " << endl;

   if (KnobDumpTranslatedCode) {
	   cerr << "Translation Cache dump:" << endl;
       dump_tc();  // dump the entire tc

	   cerr << endl << "instructions map dump:" << endl;
	   dump_entire_instr_map();     // dump all translated instructions in map_instr
   }


	// Step 6: Commit the translated routines:
	//Go over the candidate functions and replace the original ones by their new successfully translated ones:
    if (!KnobDoNotCommitTranslatedCode){
	  cout << endl;
	  commit_translated_routines();	
	  cout << "after commit translated routines\n" << endl;
    }
	else{
		cout << "not committing translated routines (-no_tc_commit)\n" << endl;
	}

	// if(DEBUG_OPT)
	// 	print_program_settings();
}


/* ===================================================================== */
/* Print Help Message                                                    */
/* ===================================================================== */
INT32 Usage()
{
    cerr << "This tool translated routines of an Intel(R) 64 binary"
         << endl;
    cerr << KNOB_BASE::StringKnobSummary();
    cerr << endl;
    return -1;
}

/* ===================================================================== */
/* prof part - print loop info into "profile.csv" file                */
/* ===================================================================== */

VOID IncCounter(UINT64& counter) { counter++; }

VOID UpdateNotSafeToInline(bool& not_safe) { not_safe = true; }

bool is_ins_not_safe_for_inline(INS ins, rtn_info_t rtnInfo)
{
			bool ret_value = false;

			// check if rtn has access to REG_RBP with positive displacement
			if(INS_MemoryBaseReg(ins) == REG_RBP && INS_MemoryDisplacement(ins) > 0)
			{
				ret_value = true;
				if(DEBUG_PROF) cout << "rtn name [" << rtnInfo.name << "], has REG_RBP ["
									<< "0x" << hex << rtnInfo.address << " " << INS_Disassemble(ins) << "]" << endl;
			}
			// check if rtn has access to REG_RSP with negative displacement
			if(INS_MemoryBaseReg(ins) == REG_RSP && INS_MemoryDisplacement(ins) < 0)
			{
				ret_value = true;
				if(DEBUG_PROF) cout << "rtn name [" << rtnInfo.name << "], has REG_RSP ["
									<< "0x" << hex << rtnInfo.address << " " << INS_Disassemble(ins) << "]" << endl;
			}
			
			// to check if function uses direct jumps outside the scope of the rtn
			if(!INS_IsRet(ins) && !INS_IsCall(ins) && INS_IsDirectControlFlow(ins)) 
			{
				ADDRINT target_rtn_address = RTN_Address(RTN_FindByAddress(INS_DirectControlFlowTargetAddress(ins)));
				if(rtnInfo.address != target_rtn_address)
				{
					rtnInfo.directJumpsMap.insert(INS_Address(ins));
					ret_value = true;
					if(DEBUG_PROF) cout << "rtn name [" << rtnInfo.name << "], has INS_IsDirectControlFlow ["
										<< "0x" << hex << rtnInfo.address << "	" << INS_Disassemble(ins) << "]" << endl;
				}
			}

			// to check if function uses indirect jumps
			if(!INS_IsRet(ins) && !INS_IsCall(ins) && INS_IsIndirectControlFlow(ins)) 
			{
				rtnInfo.indirectJumpsMap.insert(INS_Address(ins));
				ret_value = true;
				if(DEBUG_PROF) cout << "rtn name [" << rtnInfo.name << "], has INS_IsIndirectControlFlow ["
									<< "0x" << hex << rtnInfo.address << " " << INS_Disassemble(ins) << "]" << endl;
			}

			// to check if function uses calls
			if(INS_IsCall(ins)) 
			{
				rtnInfo.callingMap.insert(INS_Address(ins));
				ret_value = true;
				if(DEBUG_PROF) cout << "rtn name [" << rtnInfo.name << "], has INS_IsCall ["
									<< "0x" << hex << rtnInfo.address << " " << INS_Disassemble(ins) << "]" << endl;
			}

			// to check if function has more than one ret instructions
			if(INS_IsRet(ins))
			{
				rtnInfo.retMap.insert(INS_Address(ins));
				// ret_value = true;
			}
			
			// if the rtn is allready not safe
			if(rtnInfo.not_safe_to_inline)
				return true;

			return ret_value;
  }

VOID bbl_cond_branch_handler(bool isTaken, ADDRINT fallThroughAAdress,ADDRINT jumpAddress, bbl_info_t& bblInfo)
{
	if(isTaken)
	{
		bblInfo.countJumped++;
		bblInfo.nextBblMap[jumpAddress]++;
	}
	else
	{
		bblInfo.countFallTrhough++;
		bblInfo.nextBblMap[fallThroughAAdress]++;
	}
}

VOID bbl_normal_handler(ADDRINT fallThroughAAdress, bbl_info_t& bblInfo)
{
	bblInfo.countFallTrhough++;
	bblInfo.nextBblMap[fallThroughAAdress]++;
}

VOID bbl_uncond_branch_handler(ADDRINT jumpAddress, bbl_info_t& bblInfo)
{
	bblInfo.jumpAddress = jumpAddress;
	bblInfo.countJumped++;
	bblInfo.nextBblMap[jumpAddress]++;
}



/* ===================================================================== */
/* This function is called for every instruction executed                */
/* ===================================================================== */
VOID Trace(TRACE trace, VOID *v)
{
    for (BBL bbl = TRACE_BblHead(trace); BBL_Valid(bbl); bbl = BBL_Next(bbl))
	{
		if(!BBL_Valid(bbl))
		continue;

		bbl_info_t& bblInfo = bblMap[INS_Address(BBL_InsHead(bbl))];
		bblInfo.head_address = INS_Address(BBL_InsHead(bbl));
		bblInfo.tail_address = INS_Address(BBL_InsTail(bbl));
		bblInfo.HasFallThrough = BBL_HasFallThrough(bbl);
		bblInfo.numIns = BBL_NumIns(bbl);

        for (INS ins = BBL_InsHead(bbl); INS_Valid(ins); ins = INS_Next(ins))
		{
			RTN rtn = INS_Rtn(ins);
			if (!RTN_Valid(rtn))
				continue;
				
			if (!IMG_IsMainExecutable(IMG_FindByAddress(RTN_Address(rtn))))
				continue;

			ADDRINT rtnAddress = RTN_Address(rtn);
			ADDRINT insAddress = INS_Address(ins);

			if(routineMap.find(rtnAddress) == routineMap.end())
			{
				if(DEBUG_PROF) cout << "adding RTN [0x" << rtnAddress << "]" << endl;
			}

			// Store the routine information in the routineMap
			rtn_info_t& rtnInfo = routineMap[rtnAddress];
			rtnInfo.address = rtnAddress;
			rtnInfo.name = RTN_Name(rtn);
			rtnInfo.not_safe_to_inline = is_ins_not_safe_for_inline(ins, rtnInfo);
			rtnInfo.bblsSet.insert(bblInfo.head_address);
			// increase insCount for the current RTN
			INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)IncCounter, IARG_UINT64, &rtnInfo.insCount, IARG_END);


			// Store the bbl information in the bblMap
			bblInfo.rtnAddress = rtnAddress;
			// increase insCount for the current BBL
			INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)IncCounter, IARG_UINT64, &bblInfo.countSeen, IARG_END);

			if(insAddress == bblInfo.tail_address)
			{
				// bbl tail instruction is a branch
				if(INS_IsValidForIpointTakenBranch(ins) && INS_HasFallThrough(ins))
				{
					if(DEBUG_PROF) cout << ">>>>> [bbl_cond_branch_handler] for ins [0x"
										<< hex << insAddress << " " << INS_Disassemble(ins) << "]" << endl;

					bblInfo.jumpAddress = INS_DirectControlFlowTargetAddress(ins);
					bblInfo.nextBblMap[INS_DirectControlFlowTargetAddress(ins)] = 0;
					bblInfo.fallThroughAAdress = INS_NextAddress(ins);
					bblInfo.nextBblMap[INS_NextAddress(ins)] = 0;

					INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)bbl_cond_branch_handler,
									IARG_BRANCH_TAKEN,
									IARG_BRANCH_TARGET_ADDR,
									IARG_FALLTHROUGH_ADDR,
									IARG_PTR, &bblInfo, IARG_END);
				}
				// bbl tail instruction is a normal instruction which just falls through
				else if(!INS_IsControlFlow(ins) && INS_HasFallThrough(ins))
				{
					if(DEBUG_PROF) cout << ">>>>> [bbl_normal_handler] for ins [0x"
										<< hex << insAddress << " " << INS_Disassemble(ins) << "]" << endl;

					bblInfo.fallThroughAAdress = INS_NextAddress(ins);;
					bblInfo.nextBblMap[INS_NextAddress(ins)] = 0;

					INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)bbl_normal_handler,
									IARG_FALLTHROUGH_ADDR,
									IARG_PTR, &bblInfo, IARG_END);
				}
				// bbl tail instruction is a unconditional branch or call
				else if(INS_IsControlFlow(ins) && !INS_HasFallThrough(ins))
				{
					if(DEBUG_PROF) cout << ">>>>> [unconditional_branch_or_call] for ins [0x"
										<< hex << insAddress << " " << INS_Disassemble(ins) << "]" << endl;

					INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)bbl_uncond_branch_handler,
									IARG_BRANCH_TARGET_ADDR,
									IARG_PTR, &bblInfo, IARG_END);	
				}
				else //dont know what it is
				{
					if(DEBUG_PROF) cout << ">>>>> [unknown_tail] for ins [0x"
										<< hex << insAddress << " " << INS_Disassemble(ins) << "]" << endl;
				}
			}

			if(!INS_IsDirectCall(ins))
				continue;
			ADDRINT targetAddress = INS_DirectControlFlowTargetAddress(ins);

			// do we need this?
			// if(targetAddress > insAddress) 

			// update the callInstructionMap
			call_info_t& callInfo      		= callInstructionMap[insAddress];
			callInfo.address 				= insAddress;
			callInfo.targetAddress  		= targetAddress;
			callInfo.originRoutineName 		= RTN_FindNameByAddress(insAddress);
			callInfo.targetRoutineName 		= RTN_FindNameByAddress(targetAddress);
			callInfo.originRtn          	= &rtnInfo;
			ADDRINT targetRtnAddress 		= RTN_Address(RTN_FindByAddress(targetAddress));
			rtn_info_t& targetRtnInfo 		= routineMap[targetRtnAddress];
			callInfo.targetRtn				= &targetRtnInfo;
			INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)IncCounter, IARG_UINT64, &callInfo.countSeen, IARG_END);
			INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)IncCounter, IARG_UINT64, &targetRtnInfo.callersMap[insAddress], IARG_END);
        }
    }
}

/* ===================================================================== */
/* This function is called when the application exits	                 */
/* ===================================================================== */
VOID Fini(INT32 code, VOID *v)
{
	// Open output file
	OutFile.open(KnobOutputFile.Value().c_str());

	// Write to csv the hottest routines to reorder
	write_hottest_routines_to_csv();

	// Write to csv the hottest calls for inline
	write_hottest_calls_to_csv();

	// Write to csv the routines reordering
	write_routines_reorder_to_csv();

    OutFile.close();
}


/* ===================================================================== */
/* Main                                                                  */
/* ===================================================================== */

int main(int argc, char * argv[])
{

    // Initialize pin & symbol manager
    //out = new std::ofstream("xed-print.out");

    if(PIN_Init(argc,argv))
        return Usage();
    PIN_InitSymbols();

	if(KnobProf)
	{
		// Register the INS function
		TRACE_AddInstrumentFunction(Trace, NULL);

		// Register the exit function
		PIN_AddFiniFunction(Fini, NULL);

		// Start the program
		PIN_StartProgram();
	}

	else if(KnobOpt)
	{
		// Create the top routines map
		restore_routines_map();

		// Create the top calls map
		restore_call_map();

		// Create the reoder map
		restore_reorder_map();

		// Register ImageLoad
		IMG_AddInstrumentFunction(ImageLoad, 0);

		// Start the program, never returns
		PIN_StartProgramProbed();
	}

    return 0;
}