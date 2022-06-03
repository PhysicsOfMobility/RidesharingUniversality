

# Collective dynamics of capacity-constrained ride-pooling fleets

This is the code underlying the paper "Collective dynamics of capacity-constrained ride-pooling fleets" by R. Zech, N. Molkenthin, M. Timme, and M. Schröder.

## Code

The program simulates the dynamics of a ride-sharing fleet on a given network where requests are arbitrarily (uncorrelated origin and destination) distributed in space and Poisson-distribution in time. The dispatcher algorithm minimizes the arrival time of new requests (secondary: max pickup, tertiary: max occupancy) without delaying previously accepted requests. The simulation outputs a list of average observables as defined in README_data_file_structure.txt (see sample output).

Parameters can be set in main.cpp as a list of values the programm will loop over.

   1. Compilation
   
	  Compilation of the code requires standard C++ libraries and Boost. 
   
      The code can be compiled using CMake with the provided configuration file (including required local include directories, if necessary) using
      ````bash
      mkdir build
	  cd build
      cmake .. 
	  make
      ````

   2. Execution
      
	  All parameters for the code are set in the main.cpp. The program takes no command-line arguments. 
	  
      The code is provided in with a sample setup to run a simulation on a periodic square lattice (torus) with N = 25 nodes and B = 50 transporters at normalized request rate (load) x = 3 or the time series of the number of scheduled customers as the normalized request rate is slowly increased until x = 3. Both simulations should require only a couple of seconds to run.
	  
	  Sample output of both runs is provided as:
	  ````bash
	  min_arrival_time__uniform_requests__torus_N_25_theta_8.dat
	  ````
	  * sample output file produced by the program using only B = 50 buses and normalized request rate x = 3
	  ````bash
	  min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_timeseries.dat
	  ````
	  * sample output file of the time series output with slow ramping up of request rate


## Data 
The data folder contains final results and the data underlying the figures presented in the manuscript.
