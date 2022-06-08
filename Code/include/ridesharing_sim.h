#ifndef RIDSHARING_SIM_H
#define RIDSHARING_SIM_H

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <algorithm>
#include <utility>
#include <limits>
#include <cmath>
#include <random>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <queue>
#include <vector>

#ifndef _INTEGER_TYPES
	#define ULL uint64_t
	#define LL int64_t
	#define _INTEGER_TYPES
#endif

#include "measurement_collector.h"
#include "traffic_network.h"
#include "customer.h"
#include "transporter.h"

class customer;
class transporter;

typedef std::priority_queue< std::pair<double, ULL>, std::vector<std::pair<double, ULL> >, std::greater< std::pair<double, ULL> > > transporter_event_queue_type;

class ridesharing_sim
{
	public:
		ridesharing_sim(ULL param_N, ULL param_B, ULL seed, ULL capacity);
		virtual ~ridesharing_sim();

		void set_normalized_request_rate(double param_normalized_request_rate);
		void set_request_rate(double param_request_rate);

		void reset_number_of_buses(ULL param_number_of_buses);

		void reset_measurements();
		void print_measurements(std::ofstream& out);

		void run_sim_requests(ULL max_requests);
		void run_sim_time(long double max_time);

		double execute_next_event();

		void print_params(std::ofstream& out);

		traffic_network network;
		std::vector<transporter> transporter_list;

		double time;
		ULL total_requests;
		ULL total_serviced_requests;

		double start_of_measured_time;
		ULL start_of_measured_total_requests;
		ULL start_of_measured_serviced_requests;

		double start_of_output_time;

		void enable_measurements(double param_measurement_time_step);
		void disable_measurements();
		bool do_measurement;
		double next_measurement_time;
		double measurement_time_step;

		measurement_collector measurements;

		void enable_timeseries_output(double param_output_time_step, std::string output_filename);
		void enable_timeseries_output(double param_output_time_step);
		void disable_timeseries_output();
		bool do_timeseries_output;
		double next_output_time;
		double output_time_step;

		void output(double output_time);
		std::ofstream out;



		transporter_event_queue_type transporter_event_queue;
		double next_request_time;

		double normalized_request_rate;
		double request_rate;

		std::mt19937_64 random_generator;
		std::exponential_distribution<double> exp_dist;
};

#endif // RIDSHARING_SIM_H
