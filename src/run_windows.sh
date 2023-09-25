#!/bin/bash

# this script runs the pin tool on the bzip2 executable

RED='\033[0;31m'
GREEN='\033[;32m'
YELLOW='\033[;33m'
BLUE='\033[;34m'
MAGENTA='\033[;35m'
CYAN='\033[;36m'
RESET='\033[0m'

pin_folder_path="/mnt/c/Users/Ariel/Desktop/HASHMAL/semester6/BinaryCodeOptimization/HW/pin-folder"
project_folder_path="/mnt/c/Users/Ariel/Desktop/HASHMAL/semester6/BinaryCodeOptimization/project"

pin_program_path="$pin_folder_path/pin"
so_file_path="$project_folder_path/src/obj-intel64/project.so"

bzip2_path="$project_folder_path/given_files/bzip2"
bzip2_input_path="$project_folder_path/given_files/input-long.txt"
# bzip2_input_path="$project_folder_path/given_files/input-short.txt"

cc1_path="$project_folder_path/given_files/cc1"
cc1_input_path="$project_folder_path/given_files/200.i"
cc1_output_path="$project_folder_path/given_files/200.s"

mcf_path="$project_folder_path/given_files/mcf"
mcf_input_path="$project_folder_path/given_files/inp.in"


# Prints a string in red
print_red() {
    echo -e "\n${RED}$1${RESET}\n"
}

print_red "Running make"
make PIN_ROOT="$pin_folder_path" obj-intel64/project.so
# make PIN_ROOT=/mnt/c/Users/Ariel/Desktop/HASHMAL/semester6/BinaryCodeOptimization/HW/pin-folder obj-intel64/project.so

if [ $? -ne 0 ]; then
    echo
    print_red "make failed"
    echo
    exit 1
fi

# if ran run.sh bzip2
if [ "$1" == "bzip2" ]; then
    print_red "Running bzip2 in -prof"
    time "$pin_program_path" -t "$so_file_path" -prof -- "$bzip2_path" -k -f "$bzip2_input_path" > prof_log.txt

    print_red "Running time bzip2 in -opt -no_tc_commit"
    time "$pin_program_path" -t "$so_file_path" -opt -no_tc_commit -- "$bzip2_path" -k -f "$bzip2_input_path"

    print_red "Running time bzip2 in -opt -dump_tc"
    time "$pin_program_path" -t "$so_file_path" -opt -dump_tc -- "$bzip2_path" -k -f "$bzip2_input_path" 2> dump_tc_log.txt

    print_red "Running time bzip2 in -opt"
    time "$pin_program_path" -t "$so_file_path" -opt -- "$bzip2_path" -k -f "$bzip2_input_path"

fi

# if ran run.sh cc1
if [ "$1" == "cc1" ]; then
    print_red "Running cc1 in -prof"
    time "$pin_program_path" -t "$so_file_path" -prof -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path"

    print_red "Running time cc1 in -opt -no_tc_commit"
    time "$pin_program_path" -t "$so_file_path" -opt -no_tc_commit -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path"
    
    print_red "Running time cc1 in -opt"
    time "$pin_program_path" -t "$so_file_path" -opt -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path"

fi

# if ran run.sh mcf
if [ "$1" == "mcf" ]; then
    print_red "Running mcf in -prof"
    time "$pin_program_path" -t "$so_file_path" -prof -- "$mcf_path" "$mcf_input_path"

    print_red "Running time mcf in -opt -no_tc_commit"
    time "$pin_program_path" -t "$so_file_path" -opt -no_tc_commit -- "$mcf_path" "$mcf_input_path"

    print_red "Running time mcf in -opt"
    time "$pin_program_path" -t "$so_file_path" -opt -- "$mcf_path" "$mcf_input_path"

fi
