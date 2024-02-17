Repository must be placed in: /opt/nvidia/deepstream/deepstream-6.0/sources/apps

This is because the NVIDIA inference model makes references with paths hardcoded (offending file is labels.txt)

To compile everything, run:
$ make

To clean everything, including videos:
$ make clean

to run, issue the following:
$ ./runner.sh
