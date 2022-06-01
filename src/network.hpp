#include <boost/random.hpp>
#include <boost/random/random_number_generator.hpp>

#include <map>
#include <queue>
#include <set>

#include <cassert>

#include <boost/random.hpp>

#include "delaunay_network.h"	//generate random geometric networks

typedef boost::iterator_range< typename std::map< long, std::set<long> >::iterator > edgelist_iterator_range;
typedef boost::iterator_range< typename std::set<long>::iterator > adjacency_iterator_range;

class chain_network
{
	public:
	typedef std::map< long, std::set<long> >  edgelist_type;

	edgelist_type network_edgelist;
	long number_of_nodes;
	long number_of_links;
	long double link_probability;

	chain_network( long param_N );	//network with N nodes

	edgelist_iterator_range edges();
	adjacency_iterator_range edges(long i);
	edgelist_iterator_range network( long param_N );

	~chain_network();
	void clear();

};

chain_network::chain_network(long param_N)
{
	number_of_nodes = param_N;
	number_of_links = param_N-1;

	network(number_of_nodes);
}

edgelist_iterator_range chain_network::edges()
{
	return( boost::make_iterator_range(network_edgelist.begin(), network_edgelist.end()));
}

adjacency_iterator_range chain_network::edges(long i)
{
	return( boost::make_iterator_range(network_edgelist[i].begin(), network_edgelist[i].end()));
}

edgelist_iterator_range chain_network::network(long param_N)
{
	clear();
	number_of_nodes = param_N;
	number_of_links = param_N-1;

	for(long i = 0; i < param_N-1; ++i)		//add param_L random links
	{
		network_edgelist[i].insert(i+1);
		network_edgelist[i+1].insert(i);
	}

	return( edges() );
}

chain_network::~chain_network()
{
	clear();
}
void chain_network::clear()
{
	for(auto e : network_edgelist)
		e.second.clear();

	network_edgelist.clear();
}






template <typename generator>		//generate poisson random graphs
class poisson_random_network
{
	public:
	typedef std::map< long, std::set<long> >  edgelist_type;

	//random number distribution to create the random links
	typedef boost::random::uniform_01<long double> random_distribution_type;
	typedef boost::random::uniform_int_distribution<long> random_int_distribution_type;
	random_distribution_type uniform_dist;
	random_int_distribution_type uniform_int_dist;

	generator random_generator;

	edgelist_type network_edgelist;
	long number_of_nodes;
	long number_of_links;
	long double link_probability;

	poisson_random_network(generator &param_random_generator, long param_N, long param_L);	//network with AVERAGE L links
//	poisson_random_network(generator &param_random_generator, long param_N, long double param_p);		//network with link probability p

	edgelist_iterator_range edges();
	adjacency_iterator_range edges(long i);
	edgelist_iterator_range random_network_links(long param_N, long param_L);
	edgelist_iterator_range random_network(long param_N, long double param_p);

	~poisson_random_network();
	void clear();

};

template <typename generator>
poisson_random_network<generator>::poisson_random_network(generator &param_random_generator, long param_N, long param_L)
{
	random_generator = param_random_generator;

	number_of_nodes = param_N;

	random_network_links(number_of_nodes, param_L );
}
//template <typename generator>
//poisson_random_network<generator>::poisson_random_network(generator &param_random_generator, long param_N, long double param_p)
//{
//	random_generator = param_random_generator;
//
//	number_of_nodes = param_N;
//
//	random_network(number_of_nodes, param_p);
//}
template <typename generator>
edgelist_iterator_range poisson_random_network<generator>::edges()
{
	return( boost::make_iterator_range(network_edgelist.begin(), network_edgelist.end()));
}
template <typename generator>
adjacency_iterator_range poisson_random_network<generator>::edges(long i)
{
	return( boost::make_iterator_range(network_edgelist[i].begin(), network_edgelist[i].end()));
}
template <typename generator>
edgelist_iterator_range poisson_random_network<generator>::random_network_links(long param_N, long param_L)
{
	clear();
	number_of_nodes = param_N;
	if(param_N <= 1)
		link_probability = 0;
	else
		link_probability = 2 * param_L/( param_N * (param_N - 1));
	random_int_distribution_type::param_type params(0,number_of_nodes-1);
	uniform_int_dist.param( params );
	long i,j;

	for(long k = 0; k < param_L; ++k)		//add param_L random links
	{
		do{
			i = uniform_int_dist(random_generator);
			j = uniform_int_dist(random_generator);
		}while(i == j || network_edgelist[i].find(j) != network_edgelist[i].end() );

		network_edgelist[i].insert(j);
		network_edgelist[j].insert(i);
	}

	return( edges() );
}
template <typename generator>
edgelist_iterator_range poisson_random_network<generator>::random_network(long param_N, long double param_p)
{
	link_probability = param_p;

	clear();
	number_of_links = 0;

	long i,j;

	for(long i = 0; i < number_of_nodes; ++i)
	{
		for(long j = i + 1; j < number_of_nodes; ++j)
		{
			if( uniform_dist(random_generator) < link_probability )
			{
				network_edgelist[i].insert(j);
				network_edgelist[j].insert(i);
				++number_of_links;
			}
		}
	}

	return( edges() );
}
template <typename generator>
poisson_random_network<generator>::~poisson_random_network()
{
	clear();
}
template <typename generator>
void poisson_random_network<generator>::clear()
{
	for(auto e : network_edgelist)
		e.second.clear();

	network_edgelist.clear();
}


template <typename generator>		//generate poisson random graphs
class barabasi_albert_random_network
{
	public:
	typedef std::map< long, std::set<long> > edgelist_type;

	//random number distribution to create the random links
	typedef boost::random::uniform_01<long double> random_distribution_type;
	random_distribution_type uniform_dist;

	generator random_generator;

	edgelist_type network_edgelist;
	long number_of_nodes;
	long number_of_links;
	long initial_nodes;
	long new_node_degree;

	barabasi_albert_random_network(generator &param_random_generator, long param_N, long param_initial_nodes, long param_new_node_degree);	//network starting from fully connected initial_nodes and adding nodes with new_node degree links

	edgelist_iterator_range edges();
	adjacency_iterator_range edges(long i);
	edgelist_iterator_range random_network(long param_N, long param_initial_nodes, long param_new_node_degree);

	~barabasi_albert_random_network();
	void clear();

};

template <typename generator>
barabasi_albert_random_network<generator>::barabasi_albert_random_network(generator &param_random_generator, long param_N, long param_initial_nodes, long param_new_node_degree)
{
	number_of_nodes = param_N;
	random_generator = param_random_generator;

	if(param_new_node_degree > param_initial_nodes)
		initial_nodes = param_new_node_degree;
	else
		initial_nodes = param_initial_nodes;
	new_node_degree = param_new_node_degree;

	random_network(number_of_nodes, initial_nodes, new_node_degree);
}
template <typename generator>
edgelist_iterator_range barabasi_albert_random_network<generator>::edges()
{
	return( boost::make_iterator_range(network_edgelist.begin(), network_edgelist.end()));
}
template <typename generator>
adjacency_iterator_range barabasi_albert_random_network<generator>::edges(long i)
{
	return( boost::make_iterator_range(network_edgelist[i].begin(), network_edgelist[i].end()));
}
template <typename generator>
edgelist_iterator_range barabasi_albert_random_network<generator>::random_network(long param_N, long param_initial_nodes, long param_new_node_degree)
{
	clear();
	number_of_nodes = param_N;
	initial_nodes = param_initial_nodes;
	new_node_degree = param_new_node_degree;

	number_of_links = initial_nodes * (initial_nodes - 1);

	long random_number;
	long target_node;

	for(long i = 0; i < initial_nodes; ++i)
	{
		for(long j = i + 1; j < initial_nodes; ++j)
		{
				network_edgelist[i].insert(j);
				network_edgelist[j].insert(i);
		}
	}

	for(long i = initial_nodes; i < number_of_nodes; ++i)		//link up remaining nodes with prob. proportional to degree of nodes
	{
		for(long k = 0; k < new_node_degree; ++k)
		{
			do{
				random_number = 2*number_of_links * uniform_dist(random_generator);
				target_node = -1;
				do{
					++target_node;
					random_number -= network_edgelist[target_node].size();	//degree
				}while(target_node < i && random_number > 0);

			}while( target_node >= i || network_edgelist[i].find(target_node) != network_edgelist[i].end() );

			network_edgelist[i].insert(target_node);
			network_edgelist[target_node].insert(i);
		}

		number_of_links += new_node_degree;
	}

	return( edges() );
}
template <typename generator>
barabasi_albert_random_network<generator>::~barabasi_albert_random_network()
{
	clear();
}
template <typename generator>
void barabasi_albert_random_network<generator>::clear()
{
	for(auto e : network_edgelist)
		e.second.clear();
	network_edgelist.clear();
}

template <typename generator>		//generate poisson random graphs
class small_world_network
{
	public:
	typedef std::map< long, std::set<long> > edgelist_type;

	//random number distribution to create the random links
	typedef boost::random::uniform_01<long double> random_distribution_type;
	typedef boost::random::uniform_int_distribution<long> random_int_distribution_type;
	random_distribution_type uniform_dist;
	random_int_distribution_type uniform_int_dist;

	generator random_generator;

	edgelist_type network_edgelist;
	long number_of_nodes;
	long number_of_links;
	long double rew_probability;

	small_world_network(generator &param_random_generator, long param_N, long param_k, long L);	//network with N nodes connected to k neighbors in each direction and L additional links
//	small_world_network(generator &param_random_generator, long param_N, long param_k, long double param_p);		//network with rewiring probability p

	edgelist_iterator_range edges();
	adjacency_iterator_range edges(long i);
	edgelist_iterator_range random_network(long param_N, long param_k, long param_L);
	edgelist_iterator_range random_network(long param_N, long param_k, long double param_p);

	~small_world_network();
	void clear();

};

template <typename generator>
small_world_network<generator>::small_world_network(generator &param_random_generator, long param_N, long param_k, long param_L)
{
	random_generator = param_random_generator;

	number_of_nodes = param_N;
	number_of_links = param_N * param_k;
	rew_probability = param_L / number_of_links;

	random_network(number_of_nodes, param_k, param_L);
}
template <typename generator>
edgelist_iterator_range small_world_network<generator>::edges()
{
	return( boost::make_iterator_range(network_edgelist.begin(), network_edgelist.end()));
}
template <typename generator>
adjacency_iterator_range small_world_network<generator>::edges(long i)
{
	return( boost::make_iterator_range(network_edgelist[i].begin(), network_edgelist[i].end()));
}
template <typename generator>
edgelist_iterator_range small_world_network<generator>::random_network(long param_N, long param_k, long param_L)
{
	clear();
	number_of_nodes = param_N;
	number_of_links = param_N * param_k;
	rew_probability = param_L / number_of_links;

	number_of_links = param_N * param_k + param_L;

	long i,j;
	random_int_distribution_type::param_type params(0,number_of_nodes-1);
	uniform_int_dist.param( params );

	for(long n = 0; n < number_of_nodes; ++n)		//initial ring
	{
		for(long k = 1; k <= param_k; ++k)
		{
			network_edgelist[n].insert( (n+k) % number_of_nodes );
			network_edgelist[ (n+k) % number_of_nodes ].insert(n);
		}
	}

	for(long k = 0; k < param_L; ++k)		//for all links
	{
		do{
			i = uniform_int_dist(random_generator);
			j = uniform_int_dist(random_generator);
		}while(i == j || network_edgelist[i].find(j) != network_edgelist[i].end() );

		network_edgelist[i].insert(j);
		network_edgelist[j].insert(i);
	}

	return( edges() );
}
template <typename generator>
small_world_network<generator>::~small_world_network()
{
	clear();
}
template <typename generator>
void small_world_network<generator>::clear()
{
	for(auto e : network_edgelist)
		e.second.clear();

	network_edgelist.clear();
}

bool distances( std::map< long, std::set<long> > edgelist, long N, std::vector< std::vector<long> >& network_distances )
{
	std::queue<long> next;
	long current;

	long nodes_found;

	for(int i = 0; i < N; ++i)
	{
		network_distances[i].clear();
		network_distances[i].resize(N,-1);
		network_distances[i][i] = 0;

		nodes_found = 1;

        next.push(i);
        while(!next.empty())
		{
			current = next.front();
			next.pop();
			for( long e : edgelist[current] )
			{
				if(network_distances[i][e] == -1)
				{
					network_distances[i][e] = network_distances[i][current] + 1;
					next.push(e);
					++nodes_found;
				}
			}
		}

		if(nodes_found < N)
			return(false);
	}

	return(true);
}
