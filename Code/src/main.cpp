#include <cstddef>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <vector>
#include <cmath>
#include <random>

#include <cassert>

#include "ridesharing_sim.h"
#include "delaunay_network.h"

#ifndef _INTEGER_TYPES
	#define ULL uint64_t
	#define LL int64_t
	#define _INTEGER_TYPES
#endif

#ifndef _EPSILON
	#define MACRO_EPSILON 0.000001      //comparison of arrival times etc. is considered equal up to this difference
	#define _EPSILON
#endif

constexpr double pi = 3.14159265358979323846;

//Simulation of taxi system with arbitrary networks (needs to be strongly connected) and taxis with different service types
//everything is independent of the destination of a request (constant maximal waiting time for all requests, indiscriminate service of all requests)
int main(int argc, char* argv[])
{
	std::string topology = "torus";
	ULL number_of_nodes = 25;
	ULL capacity = 8;

	std::stringstream filename("");
	filename << "./min_arrival_time__uniform_requests__" << topology << "_N_" << number_of_nodes << "_theta_" << capacity << ".dat";
	std::ofstream out(filename.str().c_str());

//	std::vector<ULL> number_of_buses_list = {1, 2, 3, 4, 6, 8, 11, 16, 22, 32, 45, 64, 91, 128, 181, 256, 362, 512, 724, 1024, 1448, 2048};
	std::vector<ULL> number_of_buses_list = {50};
//	std::vector<double> normalized_request_rate_list = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0};
	std::vector<double> normalized_request_rate_list = {3.0};

	for(ULL number_of_buses : number_of_buses_list)
	{
		//initialize simulation class
		ridesharing_sim sim(number_of_nodes, number_of_buses, 0, capacity);

		//create topologies
		//add links for each node in the network
		for(unsigned int i = 0; i < number_of_nodes; ++i)
		{
			if(topology == "two_nodes" )
			{
				assert(number_of_nodes == 2);

				if(i == 0)
				{
					sim.network.add_link(0, 1, 1);
					sim.network.add_link(1, 0, 1);
				}
			}

			if(topology == "ring")
			{
				//create a path-graph (a line)
				if(i < number_of_nodes - 1)
				{
					sim.network.add_link(i  , i+1, 1);
					sim.network.add_link(i+1, i  , 1);
				}

				//close the ring
				if(i == number_of_nodes-1)
				{
					sim.network.add_link(0  , number_of_nodes - 1, 1);
					sim.network.add_link(number_of_nodes - 1, 0  , 1);
				}

			}

			if(topology == "torus")
			{
				//create a square lattice first
				unsigned int L = sqrt(number_of_nodes);
				if(i%L != L-1)
				{
					sim.network.add_link(i  , i+1, 1);
					sim.network.add_link(i+1, i  , 1);
				}
				if(i < L*(L-1))
				{
					sim.network.add_link(i  , i+L, 1);
					sim.network.add_link(i+L, i  , 1);
				}

				//add periodic boundaries
				if(i%L == L-1)
				{
					sim.network.add_link(i    , i+1-L, 1);
					sim.network.add_link(i+1-L, i    , 1);
				}
				if(i >= L*(L-1))
				{
					sim.network.add_link(i  , (i+L)%number_of_nodes, 1);
					sim.network.add_link((i+L)%number_of_nodes, i  , 1);
				}
			}

			if(topology == "grid")
			{
				//grid
				unsigned int L = sqrt(number_of_nodes);
				if(i%L != L-1)
				{
					sim.network.add_link(i  , i+1, 1);
					sim.network.add_link(i+1, i  , 1);
				}
				if(i < L*(L-1))
				{
					sim.network.add_link(i  , i+L, 1);
					sim.network.add_link(i+L, i  , 1);
				}
			}

			if(topology == "simplified_city")
			{
				//simplified city (spiderweb)
				unsigned int arms = sqrt(number_of_nodes);
				unsigned int nodes_per_arm = sqrt(number_of_nodes);
				unsigned int nodes_between_circles = nodes_per_arm/floor(sqrt(nodes_per_arm));
				//arms
				if( i%nodes_per_arm != 0)
				{
					sim.network.add_link(i  , i-1, 1);
					sim.network.add_link(i-1, i  , 1);
				}
				//rings
				if( (i%nodes_per_arm)%nodes_between_circles == 0 )
				{
					sim.network.add_link(i  , (i+nodes_per_arm)%number_of_nodes, 1);
					sim.network.add_link((i+nodes_per_arm)%number_of_nodes, i  , 1);
				}
			}

			if(topology == "complete_graph")
			{
				for(unsigned int j = i+1; j < number_of_nodes; ++j )
				{
					sim.network.add_link(i  , j, 1);
					sim.network.add_link(j, i  , 1);
				}
			}

			if(topology == "3cayley_tree")
			{
				for(unsigned int i = 0; i < number_of_nodes; ++i)
				{
					if(i > 0)
					{
						if(i < 4)
						{
							sim.network.add_link(i , 0, 1);
							sim.network.add_link(0 , i, 1);
						}else{
							sim.network.add_link(i, i/2-1, 1);
							sim.network.add_link(i/2-1, i, 1);
						}
					}
				}
			}

			if(topology == "delaunay_random_torus")
			{
				delaunay_network<std::mt19937_64> delaunay_net(sim.random_generator, number_of_nodes, true);
				for(auto e : delaunay_net.edges())
				{
					sim.network.add_link( e.first, e.second, delaunay_net.distance(e.first, e.second) );
					sim.network.add_link( e.second, e.first, delaunay_net.distance(e.second, e.first) );
				}
				break; //not adding edges per node
			}
		}

		//pre-calculate distance matrix (to find shortest paths later)
		sim.network.create_distances();

		//set probability distribution for origin and destination of requests
		//requests are uncorrelated from one random node to another random node
		sim.network.set_origin_probabilities();			//default: uniform
		sim.network.set_destination_probabilities();	//default: uniform
		//recompute mean distance with respect to request distribution
		sim.network.recalc_mean_distances();

		//set up (reset) all buses in the network
		for(ULL b = 0; b < number_of_buses; ++b)
		{
			sim.transporter_list[b].reset( b, sim.network.generate_request().second, capacity );
		}




		//run either of the following two simulations

		//efficiency and other measurements
		for(double normalized_request_rate : normalized_request_rate_list)
		{
			std::cout << "running simulation: " << topology << ", B = " << number_of_buses << ", x = " << normalized_request_rate << std::endl;

			sim.run_sim_requests( 1000 );
			//measure once per request per bus (on average)
			sim.enable_measurements( number_of_buses/sim.request_rate );
			sim.run_sim_requests( 1000 );

			//output results
			sim.print_params(out);
			sim.print_measurements(out);

			sim.disable_measurements();
		}





//        //timeseries output (Fig. 2a)
//        double normalized_request_rate = *(normalized_request_rate_list.begin());
//        double old_normalized_request_rate = 0;
//        double normalized_request_rate_ramp_up_step = 0.05;
//        double normalized_request_rate_ramp_up = 0;
//        ULL total_occupancy = 0;
//        ULL total_planned_stops = 0;
//
//        std::stringstream filename_detail("");\
//        filename_detail << "./min_arrival_time__uniform_requests__" << topology << "_N_" << number_of_nodes << "_B_" << number_of_buses << "_theta_" << capacity << "_timeseries.dat";
//
//        sim.enable_timeseries_output(number_of_buses/normalized_request_rate_ramp_up_step, filename_detail.str());
//
//        //set the request rate for random requests
//        for(normalized_request_rate_ramp_up = old_normalized_request_rate + normalized_request_rate_ramp_up_step; normalized_request_rate_ramp_up < normalized_request_rate + normalized_request_rate_ramp_up_step/2; normalized_request_rate_ramp_up += normalized_request_rate_ramp_up_step)
//        {
//            std::cout << "running simulation step: " << topology << ", B = " << number_of_buses << ", x = " << normalized_request_rate_ramp_up << std::endl;
//
//            sim.set_normalized_request_rate(normalized_request_rate_ramp_up);
//            sim.enable_timeseries_output(number_of_buses/sim.request_rate);
//
//            sim.run_sim_requests( std::max((ULL)1000, std::max((ULL)(normalized_request_rate_ramp_up*1000), (ULL)(5*normalized_request_rate_ramp_up*number_of_buses) ) ) );
//
//            total_occupancy = 0;
//            total_planned_stops = 0;
//
//            for(transporter& t : sim.transporter_list)
//            {
//                total_occupancy += t.get_occupancy();
//                total_planned_stops += t.get_number_of_planned_stops();
//            }
//
//            if( total_occupancy + total_planned_stops > 40 * number_of_buses * normalized_request_rate_ramp_up )
//                break;
//        }
//        if( total_occupancy + total_planned_stops > 40 * number_of_buses * normalized_request_rate_ramp_up )
//        {
//            std::cout << (total_occupancy + total_planned_stops)/(double)(2*number_of_buses) << " customers per bus at load " << normalized_request_rate_ramp_up << std::endl;
//            break;
//        }
//
//        old_normalized_request_rate = normalized_request_rate;
//        sim.set_normalized_request_rate(normalized_request_rate);
//
//        std::cout << "running simulation step: " << topology << ", B = " << number_of_buses << ", x = " << normalized_request_rate << std::endl;
//
//        sim.enable_timeseries_output(number_of_buses/sim.request_rate);
//        sim.run_sim_requests( std::max((ULL)10000, std::max((ULL)(normalized_request_rate*10000), (ULL)(normalized_request_rate*100*number_of_buses) ) ) );
//
//        //stop computing when system overloads: temporary efficiency smaller than 0.05 = 1 / 20
//        total_occupancy = 0;
//        total_planned_stops = 0;
//        for(transporter& t : sim.transporter_list)
//        {
//            total_occupancy += t.get_occupancy();
//            total_planned_stops += t.get_number_of_planned_stops();
//        }
//        if( total_occupancy + total_planned_stops > 40 * number_of_buses * normalized_request_rate )
//        {
//            std::cout << (total_occupancy + total_planned_stops)/(double)(2*number_of_buses) << " customers per bus at load " << normalized_request_rate << std::endl;
//            break;
//        }
//
//        //run the full length output if system does not overload
//        sim.run_sim_requests( std::max((ULL)100000, std::max((ULL)(normalized_request_rate*200000), (ULL)(normalized_request_rate*1000*number_of_buses) ) ) );
//        sim.disable_timeseries_output();

	}

	out.close();

	return(0);
}
