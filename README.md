# <font color="green"> Binary Translation Project - Function Inlining & Code Reordering</font>


a `pintool` that profiles a program and uses `function inlining and code reordering` to enhance performce.

## Authors

| Name              	|
|-------------------	|
| Ariel Pomeranchik 	|
| Shahar dror       	|

## JIT mode

When applied with the `-prof` knob the tool acts as a profiling Pintool (in JIT mode) that prints into a file the profiling of executed loops in each routine (RTN).

run the tool using this command :

```shell 
{path to pin} -t {path to project.so} -prof -- {path to app and inputs}
```



## Probe mode
When applied with the `-opt` knob the pintool runs in probe mode and generate the binary code of the top routines (based on total dynamic count of the instructions in the routine) according to the gathered profiling data from previous run.

The tool places the translated routines in an allocated memory area and patch them to the original image code.
`profile.csv` file should be in the same folder as project.cpp

run the tool using this command :

```shell 
{path to pin} -t {path to project.so} -opt -- {path to app and inputs}
```

## profile.csv - Explained
profile.csv file which is a output file of the `-prof` run is divided into 3 sections :

+ **Routines to reorder**
    - this section gives us the list of hot routines we want to reorder
    - countSeen is the number of time this routine ran
    - insCount is the nubmer of instructions executed in this routine
+ **Function calls to inline**
    - this section gives us the list of hot call instructions we want to inline
    - all of this calls are safe to inline and have a single strong caller
+ **Reordered blocks list for routines**
    - this section gives us the reordered blocks addresses for each routine we want to inline


profile.csv example :

```shell 
rtnAddress, rtnName, countSeen, insCount
0x401278, fallbackQSort3, 1383, 1250906
0x401090, fallbackSimpleSort, 1847, 421376

originRtnAddress, callAddress, callerRtnName, countSeen, targetAdress, targetRtnName, targetInsCount, StrongestCallerAddress
0x401278, 0x401330, fallbackQSort3, 1847, 0x401090, fallbackSimpleSort, 421376, 0x401330

rtnAddress, rtnName, orderedBBLS
0x401090, fallbackSimpleSort, 0x401090, 0x4010aa, 0x401276, 0x4010af, 0x4010c2, 0x401190, 0x40119c, 0x40126a, 0x4011aa, 0x401217, 0x40124d, 0x4010d0, 0x40113d, 0x401173, 0x40110d, 0x401145, 0x4011e7, 0x40121f, 
0x401278, fallbackQSort3, 0x401278, 0x401896, 0x4018a0, 0x4012d8, 0x4012de, 0x4012e8, 0x40133a, 0x40136f, 0x401416, 0x401436, 0x401505, 0x40150d, 0x4015dc, 0x4015e4, 0x4015ed, 0x401666, 0x401319, 0x401335, 0x4013a3, 0x4013e4, 0x4013a9, 0x40142e, 0x40143b, 0x40147c, 0x4014f4, 0x4014fa, 0x4014fc, 0x401512, 0x401553, 0x4015cb, 0x4015d1, 0x4015d3, 0x4015ef, 0x40166b, 0x401716, 0x40171c, 0x4017ca, 0x4017d0, 0x40180c, 0x401845, 0x4016a0, 0x401754, 0x401852, 
```
to print the desired information we can adjust the next thresholds inside `p_database.cpp` file:
```cpp
const float HOT_THRESHOLD_PERCENTAGE = 30.0;
const int CALL_COUNT_THRESHOLD = 100;
const int INS_COUNT_THRESHOLD = 1000;
```
+ **HOT_THRESHOLD_PERCENTAGE**
    - decides what is considered a strong caller
    - for example if the strongest caller does 100 function calls, than every other caller that does more than 30 calls is considered strong
+ **CALL_COUNT_THRESHOLD**
    - min amount of times the function run
+ **INS_COUNT_THRESHOLD**
    - min amount of instructions executed in this function
<br /><br />

# <font color="red">Example</font>


example for running the tool on bzip2 with input.txt file use the following command

```shell 
/home/pin-folder/pin -t ./obj-intel64/project.so -prof -- ./bzip2 -k -f ./input.txt
/home/pin-folder/pin -t ./obj-intel64/project.so -opt -- ./bzip2 -k -f ./input.txt
```

<br /><br />


# <font color="red">Tool Compiling</font>
to compile the library (the .so file) use pin default makefile from the folder where project.cpp is located

```shell 
cd {path to project.cpp folder}
make PIN_ROOT={path to pin folder} obj-intel64/project.so
```


for example
```shell 
make PIN_ROOT=/home/pin-folder obj-intel64/project.so
```