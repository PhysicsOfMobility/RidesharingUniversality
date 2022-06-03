#include "traffic_network.h"

traffic_network::traffic_network(ULL param_N, std::mt19937_64 &param_random_generator) : random_generator(param_random_generator)
{
	init(param_N, param_random_generator, std::vector< std::tuple<ULL, ULL, double> >() );	//call init with an empty vector of links
}

traffic_network::traffic_network(ULL param_N, std::mt19937_64 &param_random_generator, std::vector< std::tuple<ULL, ULL, double> > param_links) : random_generator(param_random_generator)
{
	init(param_N, param_random_generator, param_links);
}

//initialize the network and the relevant data structures
void traffic_network::init(ULL param_N, std::mt19937_64 &param_random_generator, std::vector< std::tuple<ULL, ULL, double> > param_links)
{
	number_of_nodes = param_N;
	uniform01 = std::uniform_real_distribution<double>(0, 1);

	potential_route_nodes.reserve(number_of_nodes);

	network_distances.resize(number_of_nodes);
    for(ULL i = 0; i < number_of_nodes; ++i)
	{
		network_distances[i] = std::vector<double>(number_of_nodes, 1e10);
	}

	//add all links in the list
	for(auto& e : param_links )
		add_link( std::get<0>(e), std::get<1>(e), std::get<2>(e) );

	//initialize default value for probabilities: uniform distribution
	origin_probabilities = std::vector<double>(number_of_nodes, 1.0 / number_of_nodes);
	random_origin = std::discrete_distribution<ULL> ( origin_probabilities.begin(), origin_probabilities.end() );
	destination_probabilities = std::vector<double>(number_of_nodes, 1.0 / number_of_nodes);
	random_destination = std::discrete_distribution<ULL> ( destination_probabilities.begin(), destination_probabilities.end() );
	recalc_mean_distances();
}

//destructor, do all the cleanup
traffic_network::~traffic_network()
{
	potential_route_nodes.clear();
	route.clear();
	origin_probabilities.clear();
	destination_probabilities.clear();

    for(ULL i = 0; i < number_of_nodes; ++i)
	{
		network_distances[i].clear();
	}
	network_distances.clear();

	for(auto& e : edgelist)
		e.second.clear();
	edgelist.clear();
}

void traffic_network::add_link( ULL from, ULL to, double dist )
{
	//assert parameter bounds
	assert(from < number_of_nodes);
	assert(to < number_of_nodes);
	assert(dist >= 0);

	//links are directed!
	edgelist[from].insert( std::make_pair(to, dist) );
}

//calculate all shortest path distances by breadth first search (direct implementation of Dijkstra)
void traffic_network::create_distances( )
{
	//reset the shape for the distance matrix, with 1e10 distances between all nodes (no distances found yet)
	network_distances.resize(number_of_nodes);
    for(ULL i = 0; i < number_of_nodes; ++i)
	{
		network_distances[i] = std::vector<double>(number_of_nodes, 1e10);
	}

	//then fill the distance matrix by finding all shortest paths (works for positive weighted graphs)
	std::priority_queue< std::pair<double, ULL>, std::vector< std::pair<double, ULL> >, std::greater< std::pair<double, ULL> > > next;
	std::pair<double, ULL> current;

	for(ULL i = 0; i < number_of_nodes; ++i)
	{
		network_distances[i][i] = 0;
        next.push( std::make_pair(network_distances[i][i], i) );

        while(!next.empty())
		{
			current = next.top();
			next.pop();

			if( network_distances[i][current.second] == current.first )
			{
				for( std::pair<ULL, double> e : edgelist[current.second] )
				{
					if(network_distances[i][e.first] > network_distances[i][current.second] + e.second)
					{
						network_distances[i][e.first] = network_distances[i][current.second] + e.second;
						next.push( std::make_pair(network_distances[i][e.first], e.first) );
					}
				}
			}
		}
	}
}

//set origin probabilities to default: uniform distribution
void traffic_network::set_origin_probabilities( )
{
	origin_probabilities = std::vector<double>(number_of_nodes, 1.0 / number_of_nodes);
	random_origin = std::discrete_distribution<ULL> ( origin_probabilities.begin(), origin_probabilities.end() );
	recalc_mean_distances();
}

//set destination probabilities to default: uniform distribution
void traffic_network::set_destination_probabilities( )
{
	destination_probabilities = std::vector<double>(number_of_nodes, 1.0 / number_of_nodes);
	random_origin = std::discrete_distribution<ULL> ( destination_probabilities.begin(), destination_probabilities.end() );
	recalc_mean_distances();
}

//set origin probabilities
void traffic_network::set_origin_probabilities( std::vector<double> param_origin_probabilities )
{
	assert(param_origin_probabilities.size() == number_of_nodes);

	origin_probabilities = param_origin_probabilities;

	//normalize probabilities
	double total_origin_prob = 0;
	for(ULL i = 0; i < number_of_nodes; ++i)
		total_origin_prob += origin_probabilities[i];
	for(ULL i = 0; i < number_of_nodes; ++i)
		origin_probabilities[i] /= total_origin_prob;

	random_origin = std::discrete_distribution<ULL> ( origin_probabilities.begin(), origin_probabilities.end() );
	recalc_mean_distances();
}

//set destination probabilities
void traffic_network::set_destination_probabilities( std::vector<double> param_dest_probabilities )
{
	assert(param_dest_probabilities.size() == number_of_nodes);

	destination_probabilities = param_dest_probabilities;

	//normalize probabilities
	double total_dest_prob = 0;
	for(ULL i = 0; i < number_of_nodes; ++i)
		total_dest_prob += destination_probabilities[i];
	for(ULL i = 0; i < number_of_nodes; ++i)
		destination_probabilities[i] /= total_dest_prob;

	random_destination = std::discrete_distribution<ULL> ( destination_probabilities.begin(), destination_probabilities.end() );
	recalc_mean_distances();
}

//compute mean distance with respect to the request distribution
void traffic_network::recalc_mean_distances()
{
	mean_pickup_distance = 0;
	mean_dropoff_distance = 0;
	for(ULL i = 0; i < number_of_nodes; ++i)
	{
		for(ULL j = 0; j < number_of_nodes; ++j)
		{
			mean_dropoff_distance += origin_probabilities[i] * destination_probabilities[j] * get_network_distance(i,j);
			mean_pickup_distance += origin_probabilities[i] * destination_probabilities[j] * get_network_distance(j,i);
		}
	}
}

//return distance in the network
double traffic_network::get_network_distance(ULL from, ULL to)
{
	return( network_distances[from][to] );
}

//generate a new request based on the (uncorrelated) origin and destination probabilities
std::pair< ULL, ULL > traffic_network::generate_request()
{
	std::pair<ULL, ULL> request;

	request.first = random_origin(random_generator);
	do{
        request.second = random_destination(random_generator);
	}while(request.second == request.first);

	return(request);
}

//return the shortest path from i to j in the form: r = ((i,t_i), (k_1,t_k_1), ... (k_n,t_k_n), (j,t_j))
//if i == j the route will have one node: r = ((i,t_i))
std::deque< std::pair<ULL, double> > traffic_network::find_shortest_path( ULL from, ULL to, double start_time, double velocity )
{
	route.clear();
	route.push_back( std::make_pair(from, start_time) );

	double temp_route_time = -1;

	ULL current_node = from;
	double current_time = start_time;
	//pick nodes until the target is reached
	while( current_node != to )
	{
		//find next nearest node on the route
		temp_route_time = 1 + get_network_distance(current_node,to) / velocity;	//set distance to something larger than possible

		potential_route_nodes.clear();
		for( auto e : edgelist[current_node] )
		{
			if( get_network_distance(e.first,to)/velocity + e.second/velocity < temp_route_time )
			{
				//if shorter distance found, clear the list of candidate nodes and add the node
				potential_route_nodes.clear();
				potential_route_nodes.push_back(e.first);
				temp_route_time = get_network_distance(e.first,to) / velocity + e.second / velocity;
			}else if( get_network_distance(e.first,to) / velocity + e.second / velocity == temp_route_time )
			{
				//add other node with the same distance to list of candidate nodes
				potential_route_nodes.push_back(e.first);
			}
		}
		//advance time along the route
		current_time += get_network_distance( current_node, potential_route_nodes[0] ) / velocity;

		//choose randomly from all possible next nodes ( NOTE: this is NOT EXATCLY THE SAME(!) as choosing randomly from all possible routes, but good enough to randomize routes in regular topologies, e.g. a torus)
		current_node = potential_route_nodes[ (ULL)( potential_route_nodes.size() * uniform01(random_generator) ) ];

		//add the node to the route
		route.push_back( std::make_pair(current_node, current_time) );
	}

	return(route);
}
