#ifndef TRAFFIC_NETWORK_H
#define TRAFFIC_NETWORK_H

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

#include <map>
#include <set>
#include <vector>
#include <tuple>
#include <queue>
#include <deque>

#include <cassert>

#ifndef _INTEGER_TYPES
	#define ULL uint64_t
	#define LL int64_t
	#define _INTEGER_TYPES
#endif

class customer;
class transporter;

struct measure{
	unsigned long n;
	long double av;
	long double stddev;

	void reset() { n=0; av=0; stddev=0;};
	void print(std::ofstream& out) { out << n << '\t' << av << '\t' << sqrt(stddev/n) << '\t'; };
};

class measurement_collector
{
	public:
		measurement_collector(traffic_network& param_network);
		virtual ~measurement_collector();

		void measure_request(customer& c, transporter& t);
		void measure_trip( std::pair<ULL, double> last_stop, std::pair<ULL, double> current_stop );
		void measure_system_status(std::vector<transporter>& transporter_list, double time);
		void reset();
		void print(std::ofstream& out);

	protected:

	private:
		long double delta1, delta2;

		measure drive_time;
		measure wait_time;
		measure delay_time;
		measure delayed_due_to_capacity;
		measure fraction_of_delayed_trips;

		measure drive_time_between_stops;
		measure drive_distance_between_stops;

		measure occupancy;
		measure scheduled_customers;
		measure number_of_planned_stops;
		measure planned_time_horizon;
		measure number_of_idle_transporters;

		traffic_network& network;

		void new_measurement(measure& m, long double val);
};

#endif // MEASUREMENTS_H
