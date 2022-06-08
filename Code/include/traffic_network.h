#ifndef TRAFFIC_NETWORK_H
#define TRAFFIC_NETWORK_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <deque>
#include <random>

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

class traffic_network
{
	public:
//		traffic_network() {};
		traffic_network(ULL param_N, std::mt19937_64 &param_random_generator);
		traffic_network(ULL param_N, std::mt19937_64 &param_random_generator, std::vector< std::tuple<ULL, ULL, double> > param_links);
		void init(ULL param_N, std::mt19937_64 &param_random_generator, std::vector< std::tuple<ULL, ULL, double> > param_links);

		virtual ~traffic_network();

		void add_link( ULL from, ULL to, double dist );
		void create_distances();

		void set_origin_probabilities( );		//set to default (uniform distribution)
		void set_destination_probabilities( );	//set to default (uniform distribution)
		void set_origin_probabilities( std::vector<double> param_probabilities );
		void set_destination_probabilities( std::vector<double> param_probabilities );

		void recalc_mean_distances();

		double get_mean_pickup_distance() { return(mean_pickup_distance); }
		double get_mean_dropoff_distance() { return(mean_dropoff_distance); }
		double get_request_asymmetry() {
											double asymmetry = 0;
											for(ULL i = 0; i < number_of_nodes; ++i)
												asymmetry += std::abs(origin_probabilities[i] - destination_probabilities[i]);

											return( asymmetry/2 );
									   }

		double get_network_distance(ULL from, ULL to);	//from i to j

		std::pair< ULL, ULL > generate_request();

		std::deque< std::pair<ULL, double> > find_shortest_path( ULL from, ULL to, double start_time, double velocity); //returns the shortest path (randomly chosen at each node if multiple options exist), !!NOT!! uniformly over all shortest paths.

	protected:

	private:
		ULL number_of_nodes;
		std::map< ULL, std::set< std::pair<ULL, double> > > edgelist;
		std::vector< std::vector<double> > network_distances; //entry [i][j] means from i to j (!!!)

		std::vector<double> origin_probabilities;
		std::vector<double> destination_probabilities;

		double mean_pickup_distance;
		double mean_dropoff_distance;

		std::discrete_distribution<ULL> random_origin;
		std::discrete_distribution<ULL> random_destination;

		std::vector< ULL > potential_route_nodes;
		std::deque< std::pair<ULL, double> > route;
		std::uniform_real_distribution<double> uniform01;

		std::mt19937_64 &random_generator;

};

#endif // TRAFFIC_NETWORK_H
