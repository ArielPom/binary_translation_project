#!/bin/bash

# this script runs the pin tool on the bzip2 executable

# Define arrays for the first and second argument options
first_args=("bzip2" "cc1" "mcf")
second_args=("no_prof")


# Function to handle tab-completion
_tab_completion() {
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}

    # Complete the first argument based on available options
    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=($(compgen -W "${first_args[*]}" -- "$cur"))
    # Complete the second argument based on the first argument
    elif [ $COMP_CWORD -eq 2 ]; then
        COMPREPLY=("$fixed_second_arg")
    else
        COMPREPLY=()
    fi
}
# Set up tab-completion for the script
complete -F _tab_completion run_linux.sh


RED='\033[0;31m'
GREEN='\033[;32m'
YELLOW='\033[;33m'
BLUE='\033[;34m'
MAGENTA='\033[;35m'
CYAN='\033[;36m'
RESET='\033[0m'

pin_folder_path="/home/ariel/university/binary_course/pin-folder"
project_folder_path="/home/ariel/university/binary_course/binary_translation_project"

pin_program_path="$pin_folder_path/pin"
so_file_path="$project_folder_path/src/obj-intel64/project.so"

bzip2_path="$project_folder_path/given_files/bzip2"
bzip2_input_path="$project_folder_path/given_files/input-short.txt"
# bzip2_input_path="$project_folder_path/given_files/input-long.txt"

cc1_path="$project_folder_path/given_files/cc1"
cc1_input_path="$project_folder_path/given_files/200.i"
cc1_output_path="$project_folder_path/given_files/200.s"

mcf_path="$project_folder_path/given_files/mcf"
mcf_input_path="$project_folder_path/given_files/inp.in"

output1=""
output2=""


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
    if [ "$2" != "no_prof" ]; then
        print_red "Running bzip2 in -prof"
        time "$pin_program_path" -t "$so_file_path" -prof -- "$bzip2_path" -k -f "$bzip2_input_path" > prof_out.txt
    fi

    print_red "Running perf stat on bzip2 in -opt -no_tc_commit"
    output1=$(sudo perf stat -d -d -d -r 5 --table "$pin_program_path" -t "$so_file_path" -opt -no_tc_commit -- "$bzip2_path" -k -f "$bzip2_input_path" 2>&1)
    echo "$output1"

    print_red "Running perf stat on bzip2 in -opt -dump_tc -verbose"
    sudo perf stat  "$pin_program_path" -t "$so_file_path" -opt -dump_tc -verbose -- "$bzip2_path" -v -v -k -f "$bzip2_input_path" 2> dump_tc_bzip2_log.txt

    print_red "Running perf stat on bzip2 in -opt"
    output2=$(sudo perf stat -d -d -d -r 5 --table "$pin_program_path" -t "$so_file_path" -opt -- "$bzip2_path" -k -f "$bzip2_input_path" 2>&1)
    echo "$output2"
fi

# if ran run.sh cc1
if [ "$1" == "cc1" ]; then

    if [ "$2" != "no_prof" ]; then
        print_red "Running cc1 in -prof"
        time "$pin_program_path" -t "$so_file_path" -prof -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path"
    fi

    print_red "Running perf stat on cc1 in -opt -no_tc_commit"
    output1=$(sudo perf stat -d -d -d "$pin_program_path" -t "$so_file_path" -opt -no_tc_commit -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path" 2>&1)
    echo "$output1"

    print_red "Running perf stat on cc1 in -opt -dump_tc -verbose"
    sudo perf stat  "$pin_program_path" -t "$so_file_path" -opt -dump_tc -verbose -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path" 2> dump_tc_cc1_log.txt

    print_red "Running perf stat on cc1 in -opt"
    output2=$(sudo perf stat -d -d -d "$pin_program_path" -t "$so_file_path" -opt -- "$cc1_path" "$cc1_input_path" -o "$cc1_output_path" 2>&1)
    echo "$output2"
fi

# if ran run.sh mcf
if [ "$1" == "mcf" ]; then

    if [ "$2" != "no_prof" ]; then
    print_red "Running mcf in -prof"
    time "$pin_program_path" -t "$so_file_path" -prof -- "$mcf_path" "$mcf_input_path" > prof_out.txt
    fi

    print_red "Running perf stat on mcf in -opt -no_tc_commit"
    output1=$(sudo perf stat "$pin_program_path" -t "$so_file_path" -opt -no_tc_commit -- "$mcf_path" "$mcf_input_path" 2>&1)
    echo "$output1"

    print_red "Running perf stat on mcf in -opt -dump_tc -verbose"
    sudo perf stat  "$pin_program_path" -t "$so_file_path" -opt -dump_tc -verbose -- "$mcf_path" "$mcf_input_path" 2> dump_tc_mcf_log.txt

    print_red "Running perf stat on mcf in -opt"
    output2=$(sudo perf stat "$pin_program_path" -t "$so_file_path" -opt -- "$mcf_path" "$mcf_input_path" 2>&1)
    echo "$output2"
fi



output_file="comparison.csv"

# Check if the file exists and delete it if it does
if [ -e "$output_file" ]; then
    rm "$output_file"
fi

# Function to extract values from perf stat output
extract_metric_values() {
    grep -E "$1" | awk '!/after/ {gsub(/,/, ""); print $1; exit}'
}

# Function to format a number with commas every 3 digits
format_number_with_commas() {
    local formatted_number=""
    if [[ "$1" =~ \. ]]; then
        # If the number has a fractional part, format it without trailing zeros
        formatted_number=$(printf "%s" "$1" | awk '{sub(/\.?0*$/,"");} 1')
    else
        # If the number is an integer, format the integer part without decimal fractions
        formatted_number=$(printf "%'d" "$1")
    fi
    echo "$formatted_number"
}

# Extract values for each metric
metrics=("context-switches" "branches" "branch-misses" "L1-icache-load-misses" "iTLB-load-misses" "time elapsed")

# Initialize the CSV file with headers
echo "Metric,no_tc_commit,tc_commit" > comparison.csv

# Loop through each metric, extract values, and save to CSV
for metric in "${metrics[@]}"; do
    values1=$(format_number_with_commas "$(echo "$output1" | tail -n 30 | extract_metric_values "$metric")")
    values2=$(format_number_with_commas "$(echo "$output2" | tail -n 30 | extract_metric_values "$metric")")
    echo "$metric,\"$values1\",\"$values2\"" >> "$output_file"
done