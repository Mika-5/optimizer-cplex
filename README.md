optimizer-cplex
===================

Compute an optimized solution to the Vehicle Routing Problem using CPOptimizer.

Installation
============

## Optimizer

Require CPLEX.

Compile the C++ optimizer

    make third_party
    make tsp_cplex

Test
====

LD_LIBRARY_PATH=../optimizer-cplex/dependencies/install/lib/:../CPLEX/cpoptimizer/lib/x86-64/static_pic/ ../optimizer-cplex/tsp_cplex  -instance_file 'data/3missions_2vehicles_test' -solution_file 'optimize-cplex-output3m2v_test'
