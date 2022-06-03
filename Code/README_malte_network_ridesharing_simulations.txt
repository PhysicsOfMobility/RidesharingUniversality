The program simulates the dynamics of a ride-sharing fleet on a given network where requests are arbitrarily (uncorrelated origin and destination) distributed in space and Poisson-distribution in time. The dispatcher algorithm minimizes the arrival time of new requests (secondary: max pickup, tertiary: max occupancy) without delaying previously accepted requests. The simulation outputs a list of average observables as defined in README_data_file_structure.txt (see sample output).

Parameters can be set in main.cpp as a list of values the programm will loop over.


FILES:

	main.cpp		(main file, calls the simulation with given parameters [no command line input])

	ridesharing_sim.h
	ridesharing_sim.cpp	(contains most of the event based simulation)

	traffic_network.h
	traffic_network.cpp	(contains all network things, shortest path, distances, etc.)

	transporter.h
	transporter.cpp		(contains the dispatcher/assignmnet algorithm & transporter parts of the event based simulation)

	customer.h
	customer.cpp		(helper to track individual customer requests and their delay)

	measurement_collector.h
	measurement_collector.cpp	(helper to collect measurements for average and standard deviation of observables)



SAMPLE OUTPUT:
	min_arrival_time__uniform_requests__torus_N_25_theta_8.dat	(sample output file produced by the program using only B = 50 buses and normalized request rate x = 3)
	min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_timeseries   (sample output file of the time series output with slow ramping up of request rate)


