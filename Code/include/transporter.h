#ifndef TRANSPORTER_H
#define TRANSPORTER_H

#include <cstdlib>
#include <algorithm>
#include <limits>
#include <random>
#include <iostream>
#include <deque>
#include <list>
#include <cmath>
#include <cassert>

#ifndef _INTEGER_TYPES
	#define ULL uint64_t
	#define LL int64_t
	#define _INTEGER_TYPES
#endif

#ifndef _EPSILON
	#define MACRO_EPSILON 0.000000000001
	#define _EPSILON
#endif

#include "measurement_collector.h"
#include "customer.h"
#include "traffic_network.h"

class measurement_collector;
class customer;
class transporter;

struct stop
{
	ULL node_index;

	std::list<customer>::iterator c_it;
	double planned_time;

	bool is_pickup;
	bool is_dropoff;

	ULL type;

	stop( ULL param_node_index, std::list<customer>::iterator param_customer, double param_time, bool param_is_pickup, bool param_is_dropoff, ULL param_type)
		: 	node_index(param_node_index),
			c_it(param_customer),
			planned_time(param_time),
			is_pickup(param_is_pickup),
			is_dropoff(param_is_dropoff),
			type(param_type)
		{};
};

struct offer
{
	ULL transporter_index;
	transporter* best_transporter;

	double pickup_time;
	std::list< stop >::iterator pickup_insertion;

	double dropoff_time;
	std::list< stop >::iterator dropoff_insertion;

	bool is_better_offer;

	offer( ULL param_transporter_index, transporter* param_best_transporter, double param_pickup_time, std::list< stop >::iterator param_pickup_insertion, double param_dropoff_time, std::list< stop >::iterator param_dropoff_insertion)
		:	transporter_index(param_transporter_index),
			best_transporter(param_best_transporter),
			pickup_time(param_pickup_time),
			pickup_insertion(param_pickup_insertion),
			dropoff_time(param_dropoff_time),
			dropoff_insertion(param_dropoff_insertion),
			is_better_offer(false)
		{};

    offer() : dropoff_time( std::numeric_limits<double>::infinity()), is_better_offer(false)  {};

};

class transporter
{
	public:
		transporter(ULL param_index, ULL param_location, ULL param_type, std::mt19937_64 param_random_generator);
		void reset(ULL param_index, ULL param_location, ULL param_type);
		void init_by_type();
		virtual ~transporter();

		ULL get_index() { return(index); }
		LL get_capacity() { return(capacity); }
		double get_velocity() { return(velocity); }
		ULL get_type() { return(type); }

		ULL get_current_location() { return(current_location); }
		double get_current_time() { return(current_time); }

		LL get_occupancy() { return(occupancy); }
		LL get_number_of_scheduled_customers() { return(assigned_customers.size()); }
		bool is_idle() { return(idle); }

		std::pair<ULL, double> get_next_route_node() { return( current_route.front() ); }	//should always be the same as current position and time (!!!)

		ULL get_number_of_planned_stops() { assert( assigned_stops.size() == 2*assigned_customers.size() - occupancy );
			return( assigned_stops.size() ); };
		double get_planned_time_horizon(double time) {
			if(assigned_stops.empty())
				return(0);
			else
				return( assigned_stops.rbegin()->planned_time - time );
		}
		std::list< stop >::iterator get_next_stop() { return( assigned_stops.begin() ); }
		std::list< stop >::iterator no_stop() { return( assigned_stops.end() ); }

		double execute_event(double time, traffic_network& n, measurement_collector& m, ULL& total_serviced_requests, bool do_measurement);	//execute event, returns next event time (if any)
		double new_route( const std::deque< std::pair<ULL, double> > &param_new_route );

		offer best_offer( ULL param_origin, ULL param_destination, double param_request_time, traffic_network &n, offer& current_best_offer );
		offer best_offer_unlimited( ULL param_origin, ULL param_destination, double param_request_time, traffic_network &n, offer& current_best_offer );
		double assign_customer(double assignment_time, customer c, offer& o, traffic_network &n);

	protected:

	private:
		ULL index;

		LL capacity;		//negative value --> infinite capacity
		double velocity;
		ULL type;
		//add arbitrary parameters how

		//next node to be at and when the transporter arrives
		ULL current_location;
		double current_time;
		ULL node_of_last_stop;
		double time_of_last_stop;


		std::deque< std::pair<ULL, double> > current_route;	//list of nodes on the route to the next stop
		std::list< stop > assigned_stops;

		std::list<customer> assigned_customers;

		LL occupancy;
		bool idle;

		std::mt19937_64 &random_generator;
};

#endif // TRANSPORTER_H
