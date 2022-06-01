/*#include "delaunay_network.h"



//constructor for empty network
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::delaunay_network(generator& param_random_generator)
{
	state = empty;
	max_position_value = std::numeric_limits<int_coordinate_type>::max() / 4;
	uniform_dist.param(typename random_distribution_type::param_type(0,max_position_value));

	periodic = false;
	number_of_points = 0;
	random_generator = param_random_generator;
}

//constructor for delaunay network (same as above but also calls "random_delaunay_network(...)"
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::delaunay_network(generator &param_random_generator, network_size_type param_N, bool param_periodic)
{
	state = empty;
	max_position_value = std::numeric_limits<int_coordinate_type>::max() / 4;
	uniform_dist.param(typename random_distribution_type::param_type(0,max_position_value));

	periodic = false;
	number_of_points = 0;
	random_generator = param_random_generator;

	random_delaunay_network(param_N, param_periodic);
}

template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::~delaunay_network()
{
	clear();
}

//clear the network and reset (also used before destruction
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
void delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::clear()
{
	int_positions.clear();
	positions_wrapped.clear();
	network_edgelist.clear();

	vd.clear();

	state = empty;

	periodic = false;
	number_of_points = 0;
}

//get the edgelist (via iterator range) to use the network data
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::edgelist_iterator_range delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::edges()
{
	return( boost::make_iterator_range(network_edgelist.begin(), network_edgelist.end()));
}

//get position (in unit square) of  point i (if it exists)
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::float_point delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::position(network_size_type i)
{
	if( i >= 0 && i < number_of_points)
		return( float_point( int_positions[i].x()/(float_coordinate_type)max_position_value, int_positions[i].y()/(float_coordinate_type)max_position_value ) );
	else
		return( float_point( std::numeric_limits<float_coordinate_type>::quiet_NaN(), std::numeric_limits<float_coordinate_type>::quiet_NaN() ));
}

template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::float_point delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::wrapped_position(network_size_type i)
{
	if( i >= 0 && i < 9*number_of_points)
		return( float_point( positions_wrapped[i].x()/(float_coordinate_type)max_position_value, positions_wrapped[i].y()/(float_coordinate_type)max_position_value ) );
	else
		return( float_point( std::numeric_limits<float_coordinate_type>::quiet_NaN(), std::numeric_limits<float_coordinate_type>::quiet_NaN() ));
}

//create random network by creating a triangulation of N points in the [0,1]x[0,1] unit square
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::edgelist_iterator_range delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::random_delaunay_network(network_size_type param_N, bool param_periodic)
{
	if(param_N == number_of_points && param_periodic == periodic)
	{
		return( random_delaunay_network() );
	}else if(param_periodic == false)
	{
		int_positions.resize(param_N);
		number_of_points = param_N;
		periodic = param_periodic;


		for(int_position& p : int_positions)
		{
			p = int_position(uniform_dist(random_generator), uniform_dist(random_generator));
		}
		positions_wrapped.clear();
		network_edgelist.clear();

		boost::polygon::construct_voronoi(int_positions.begin(), int_positions.end(), &vd);

		//simply add all edges from the delaunay triangulation to the list (as this is the most connected network, all other networks contain the same or fewer edges)
		for(const typename voronoi_type::edge_type &v_edge : vd.edges())
		{
			if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
				network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
			else
				network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
		}

		state = delaunay_network::delaunay;
		return( edges() );
	}else{ //periodic = true

		int_positions.resize(param_N);
		number_of_points = param_N;
		periodic = param_periodic;


		for(int_position& p : int_positions)
		{
			p = int_position(uniform_dist(random_generator), uniform_dist(random_generator));
		}

		positions_wrapped.resize(9*number_of_points);
		for(network_size_type i = 0; i < number_of_points; ++i)
		{
			positions_wrapped[i] = int_positions[i];
			positions_wrapped[number_of_points+i] = int_position(int_positions[i].x() + max_position_value, int_positions[i].y());
			positions_wrapped[2*number_of_points+i] = int_position(int_positions[i].x(), int_positions[i].y() + max_position_value);
			positions_wrapped[3*number_of_points+i] = int_position(int_positions[i].x() + max_position_value, int_positions[i].y() + max_position_value);
			positions_wrapped[4*number_of_points+i] = int_position(int_positions[i].x() - max_position_value, int_positions[i].y());
			positions_wrapped[5*number_of_points+i] = int_position(int_positions[i].x(), int_positions[i].y() - max_position_value);
			positions_wrapped[6*number_of_points+i] = int_position(int_positions[i].x() - max_position_value, int_positions[i].y() - max_position_value);
			positions_wrapped[7*number_of_points+i] = int_position(int_positions[i].x() - max_position_value, int_positions[i].y() + max_position_value);
			positions_wrapped[8*number_of_points+i] = int_position(int_positions[i].x() + max_position_value, int_positions[i].y() - max_position_value);
		}

		boost::polygon::construct_voronoi(positions_wrapped.begin(), positions_wrapped.end(), &vd);

		network_edgelist.clear();

		//simply add all edges from the delaunay triangulation to the list (as this is the most connected network, all other networks contain the same or fewer edges)
		for(const typename voronoi_type::edge_type &v_edge : vd.edges())
		{
			if(v_edge.cell()->source_index() < number_of_points || v_edge.twin()->cell()->source_index() < number_of_points)
			{
				if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
					network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
				else
					network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
			}
		}

		state = delaunay_network::delaunay;
		return( edges() );
	}
}

template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::edgelist_iterator_range delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::random_delaunay_network()
{
	if(periodic == false)
	{
		if(state == delaunay_network::delaunay || state == delaunay_network::empty)
		{
			return( edges() );
		}else{
			network_edgelist.clear();
			//simply add all edges from the delaunay triangulation to the list (as this is the most connected network, all other networks contain the same or fewer edges)
			for(const typename voronoi_type::edge_type &v_edge : vd.edges())
			{
				if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
					network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
				else
					network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
			}

			state = delaunay_network::delaunay;
			return( edges() );
		}

	}else{ //periodic = true
		if(state == delaunay_network::delaunay || state == delaunay_network::empty)
		{
			return( edges() );
		}else{
			network_edgelist.clear();
			//simply add all edges from the delaunay triangulation to the list (as this is the most connected network, all other networks contain the same or fewer edges)
			for(const typename voronoi_type::edge_type &v_edge : vd.edges())
			{
				if(v_edge.cell()->source_index() < number_of_points || v_edge.twin()->cell()->source_index() < number_of_points)
				{
					if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
						network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
					else
						network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
				}
			}

			state = delaunay_network::delaunay;
			return( edges() );
		}
	}
}

//////create the gabriel graph for a set of random points (or from the already existing network [also done if arguments are identical], for a new network, need an explicit clear()
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::edgelist_iterator_range delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::random_gabriel_network(network_size_type param_N, bool param_periodic)
{
	if(param_N == number_of_points && param_periodic == periodic)
	{
		return( random_gabriel_network () );
	}else if(param_periodic == false)
	{
		int_positions.resize(param_N);
		number_of_points = param_N;
		periodic = param_periodic;

		for(int_position& p : int_positions)
		{
			p = int_position(uniform_dist(random_generator), uniform_dist(random_generator));
		}
		positions_wrapped.clear();

		boost::polygon::construct_voronoi(int_positions.begin(), int_positions.end(), &vd);
		network_edgelist.clear();

		float_point u;
		float_point v;
		//check all edges of the triangulation and add the ones belonging to the gabriel graph
		for(const typename voronoi_type::edge_type &v_edge : vd.edges())
		{
			u = position(v_edge.cell()->source_index());
			v = position(v_edge.twin()->cell()->source_index());

			if(v_edge.is_infinite())
			{
				float_point direction = float_point( u.x()-v.x(), u.y()-v.y() );
				u = float_point( u.x() + 100*direction.x(), u.y() + 100*direction.y() );
				v = float_point( v.x() - 100*direction.x(), v.y() - 100*direction.y() );

				if(v_edge.vertex1() == NULL)
				{
					//check intersection of extended segment and segment from center of unit square to vertex
					//if no intersection, vertex is inside the convex hull of the point set and the voronoi edge going to infinity will intersect -> part of gabriel graph
					if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
					{
						if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
							network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
						else
							network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
					}
				}else{ //vertex0() == NULL
					if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
					{
						if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
							network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
						else
							network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
					}
				}
			}else{
				//if distance to the other vertices forming triangles is larger than half the distance from vertex to vertex, add edge
				if(   boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value) )  )   )
				{
					if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
						network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
					else
						network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
				}
			}
		}

		state = delaunay_network::gabriel;
		return( edges() );
	}else{ //periodic = true

		int_positions.resize(param_N);
		number_of_points = param_N;
		periodic = param_periodic;

		for(int_position& p : int_positions)
		{
			p = int_position(uniform_dist(random_generator), uniform_dist(random_generator));
		}

		positions_wrapped.resize(9*number_of_points);
		for(network_size_type i = 0; i < number_of_points; ++i)
		{
			positions_wrapped[i] = int_positions[i];
			positions_wrapped[number_of_points+i] = int_position(int_positions[i].x() + max_position_value, int_positions[i].y());
			positions_wrapped[2*number_of_points+i] = int_position(int_positions[i].x(), int_positions[i].y() + max_position_value);
			positions_wrapped[3*number_of_points+i] = int_position(int_positions[i].x() + max_position_value, int_positions[i].y() + max_position_value);
			positions_wrapped[4*number_of_points+i] = int_position(int_positions[i].x() - max_position_value, int_positions[i].y());
			positions_wrapped[5*number_of_points+i] = int_position(int_positions[i].x(), int_positions[i].y() - max_position_value);
			positions_wrapped[6*number_of_points+i] = int_position(int_positions[i].x() - max_position_value, int_positions[i].y() - max_position_value);
			positions_wrapped[7*number_of_points+i] = int_position(int_positions[i].x() - max_position_value, int_positions[i].y() + max_position_value);
			positions_wrapped[8*number_of_points+i] = int_position(int_positions[i].x() + max_position_value, int_positions[i].y() - max_position_value);
		}

		boost::polygon::construct_voronoi(positions_wrapped.begin(), positions_wrapped.end(), &vd);
		network_edgelist.clear();

		float_point u;
		float_point v;
		//check all edges of the triangulation and add the ones belonging to the gabriel graph
		for(const typename voronoi_type::edge_type &v_edge : vd.edges())
		{
			if(v_edge.cell()->source_index() < number_of_points || v_edge.twin()->cell()->source_index() < number_of_points)
			{
				u = wrapped_position(v_edge.cell()->source_index());
				v = wrapped_position(v_edge.twin()->cell()->source_index());

				if(v_edge.is_infinite())
				{
					float_point direction = float_point( u.x()-v.x(), u.y()-v.y() );
					u = float_point( u.x() + 100*direction.x(), u.y() + 100*direction.y() );
					v = float_point( v.x() - 100*direction.x(), v.y() - 100*direction.y() );

					if(v_edge.vertex1() == NULL)
					{
						//check intersection of extended segment and segment from center of unit square to vertex
						//if no intersection, vertex is inside the convex hull of the point set and the voronoi edge going to infinity will intersect -> part of gabriel graph
						if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
						{
							if( v_edge.cell()->source_index() % number_of_points < v_edge.twin()->cell()->source_index() % number_of_points )
								network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
							else
								network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
						}
					}else{ //vertex0() == NULL
						if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
						{
							if( v_edge.cell()->source_index() % number_of_points < v_edge.twin()->cell()->source_index() % number_of_points )
								network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
							else
								network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
						}
					}
				}else{
					//if distance to the other vertices forming triangles is larger than half the distance from vertex to vertex, add edge
					if(   boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value) )  )   )
					{
						if( v_edge.cell()->source_index() % number_of_points < v_edge.twin()->cell()->source_index() % number_of_points )
								network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
							else
								network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
					}
				}
			}
		}
		return( edges() );
	}
}
template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
typename delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::edgelist_iterator_range delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::random_gabriel_network()
{
	if(periodic == false)
	{
		if(state == delaunay_network::gabriel || state == delaunay_network::empty)
		{
			return( edges() );
		}else{		//state = delaunay
			network_edgelist.clear();

			float_point u;
			float_point v;
			//check all edges of the triangulation and add the ones belonging to the gabriel graph
			for(const typename voronoi_type::edge_type &v_edge : vd.edges())
			{
				u = position(v_edge.cell()->source_index());
				v = position(v_edge.twin()->cell()->source_index());

				if(v_edge.is_infinite())
				{
					float_point direction = float_point( u.x()-v.x(), u.y()-v.y() );
					u = float_point( u.x() + 100*direction.x(), u.y() + 100*direction.y() );
					v = float_point( v.x() - 100*direction.x(), v.y() - 100*direction.y() );

					if(v_edge.vertex1() == NULL)
					{
						//check intersection of extended segment and segment from center of unit square to vertex
						//if no intersection, vertex is inside the convex hull of the point set and the voronoi edge going to infinity will intersect -> part of gabriel graph
						if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
						{
							if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
								network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
							else
								network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
						}
					}else{ //vertex0() == NULL
						if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
						{
							if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
								network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
							else
								network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
						}
					}
				}else{
					//if distance to the other vertices forming triangles is larger than half the distance from vertex to vertex, add edge
					if(   boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value) )  )   )
					{
						if( v_edge.cell()->source_index() < v_edge.twin()->cell()->source_index() )
							network_edgelist.insert( std::make_pair( v_edge.cell()->source_index(), v_edge.twin()->cell()->source_index() ) );
						else
							network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index(), v_edge.cell()->source_index() ) );
					}
				}
			}

			state = delaunay_network::gabriel;
			return( edges() );
		}

	}else{ //periodic = true
		if(state == delaunay_network::gabriel || state == delaunay_network::empty)
		{
			return( edges() );
		}else{		//state = delaunay
			network_edgelist.clear();

			float_point u;
			float_point v;
			//check all edges of the triangulation and add the ones belonging to the gabriel graph
			for(const typename voronoi_type::edge_type &v_edge : vd.edges())
			{
				if(v_edge.cell()->source_index() < number_of_points || v_edge.twin()->cell()->source_index() < number_of_points)
				{
					u = wrapped_position(v_edge.cell()->source_index());
					v = wrapped_position(v_edge.twin()->cell()->source_index());

					if(v_edge.is_infinite())
					{
						float_point direction = float_point( u.x()-v.x(), u.y()-v.y() );
						u = float_point( u.x() + 100*direction.x(), u.y() + 100*direction.y() );
						v = float_point( v.x() - 100*direction.x(), v.y() - 100*direction.y() );

						if(v_edge.vertex1() == NULL)
						{
							//check intersection of extended segment and segment from center of unit square to vertex
							//if no intersection, vertex is inside the convex hull of the point set and the voronoi edge going to infinity will intersect -> part of gabriel graph
							if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
							{
								if( v_edge.cell()->source_index() % number_of_points < v_edge.twin()->cell()->source_index() % number_of_points )
									network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
								else
									network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
							}
						}else{ //vertex0() == NULL
							if(   !boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value),float_point(0.5,0.5) )  )   )
							{
								if( v_edge.cell()->source_index() % number_of_points < v_edge.twin()->cell()->source_index() % number_of_points )
									network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
								else
									network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
							}
						}
					}else{
						//if distance to the other vertices forming triangles is larger than half the distance from vertex to vertex, add edge
						if(   boost::geometry::intersects(  Segment( u,v )  ,  Segment( float_point(v_edge.vertex0()->x()/(float_coordinate_type)max_position_value,v_edge.vertex0()->y()/(float_coordinate_type)max_position_value),float_point(v_edge.vertex1()->x()/(float_coordinate_type)max_position_value,v_edge.vertex1()->y()/(float_coordinate_type)max_position_value) )  )   )
						{
							if( v_edge.cell()->source_index() % number_of_points < v_edge.twin()->cell()->source_index() % number_of_points )
								network_edgelist.insert( std::make_pair( v_edge.cell()->source_index() % number_of_points, v_edge.twin()->cell()->source_index() % number_of_points ) );
							else
								network_edgelist.insert( std::make_pair( v_edge.twin()->cell()->source_index() % number_of_points, v_edge.cell()->source_index() % number_of_points ) );
						}
					}
				}
			}
			state = delaunay_network::gabriel;
			return( edges() );
		}
	}
}


//////create the relative neighborhood graph for a set of random points (or from the already existing network [also done if arguments are identical], for a new network, need an explicit clear()
////edgelist_iterator_rangerandom_relative_neighborhood_network(network_size_type param_N, bool param_periodic);
////edgelist_iterator_rangerandom_relative_neighborhood_network();


template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
float_coordinate_type delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::voronoi_area(network_size_type i)
{
	if(state == delaunay_network::empty)
		return(0);

	if(periodic)
	{
		if(i >= 0 && i < number_of_points)
		{
			typename voronoi_type::cell_container_type::const_iterator c = vd.cells().begin();

			while( c != vd.cells().end() && c->source_index() != i )
				++c;


			if( c == vd.cells().end() || c->source_index() != i )
			{
				std::cout << "source index error" << '\t' << i << '\t' << c->source_index() << std::endl;
				return(0);
			}


			const typename voronoi_type::edge_type *v_edge = c->incident_edge();
			const typename voronoi_type::edge_type *v_edge_ccw = v_edge;

			boost::geometry::model::polygon< float_point, false, false > polycell;
			do{
				if( v_edge_ccw->vertex0() != NULL)
					boost::geometry::append(polycell.outer(), float_point(v_edge_ccw->vertex0()->x()/(float_coordinate_type)max_position_value, v_edge_ccw->vertex0()->y()/(float_coordinate_type)max_position_value) );

				v_edge_ccw = v_edge_ccw->next();
			}while( v_edge_ccw != v_edge );

			return(boost::geometry::area(polycell));
		}else{
			return(0);
		}
	}else{
		//TODO!!!!	only doing area within the convex hull for now (i.e. ignoring infinite edges of the cells)
		if(i >= 0 && i < number_of_points)
		{
			typename voronoi_type::cell_container_type::const_iterator c = vd.cells().begin();

			while( c != vd.cells().end() && c->source_index() != i )
				++c;


			if( c == vd.cells().end() || c->source_index() != i )
			{
				std::cout << "source index error" << '\t' << i << '\t' << c->source_index() << std::endl;
				return(0);
			}


			const typename voronoi_type::edge_type *v_edge = c->incident_edge();
			const typename voronoi_type::edge_type *v_edge_ccw = v_edge;

			boost::geometry::model::polygon< float_point, false, false > polycell;
			do{
				if( v_edge_ccw->vertex0() != NULL)
					boost::geometry::append(polycell.outer(), float_point(v_edge_ccw->vertex0()->x()/(float_coordinate_type)max_position_value, v_edge_ccw->vertex0()->y()/(float_coordinate_type)max_position_value) );

				v_edge_ccw = v_edge_ccw->next();
			}while( v_edge_ccw != v_edge );

			return(boost::geometry::area(polycell));
		}else{
			return(0);
		}
	}
}

template <typename generator, typename network_size_type, typename int_coordinate_type, typename float_coordinate_type>
float_coordinate_type delaunay_network<generator, network_size_type, int_coordinate_type, float_coordinate_type>::distance( network_size_type i, network_size_type j)
{
	if(state == delaunay_network::empty)
		return(0);

	if(periodic)
	{
		if( i < 0 || j < 0 || i >= number_of_points || j >= number_of_points)
			return(0);

		float_coordinate_type dist = 2;
		float_coordinate_type temp_dist = 0;

		for( int k = 0; k < 9; ++k)	//check distance to every image of the node j
		{
			temp_dist = sqrt( pow( wrapped_position(i).x() - wrapped_position(j+number_of_points*k).x() ,2) + pow( wrapped_position(i).y() - wrapped_position(j+number_of_points*k).y() ,2) );
			if(temp_dist < dist)
				dist = temp_dist;
		}
		return(dist);
	}else{
		if( i < 0 || j < 0 || i >= number_of_points || j >= number_of_points)
			return(0);

		return( sqrt( pow( position(i).x() - position(j).x() ,2) + pow( position(i).y() - position(j).y() ,2) ) );
	}
}

*/
