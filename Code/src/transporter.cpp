#include "transporter.h"

//constructor, initialize all necessary variables
transporter::transporter(ULL param_index, ULL param_location, ULL param_type, std::mt19937_64 param_random_generator) : random_generator(param_random_generator)
{
	//bus starts in a given location with no next arrival (current time)
	index = param_index;
	current_location = param_location;
	current_time = -1;

	//bus has no current route
	node_of_last_stop = current_location;
	time_of_last_stop = -1;

	//bus is initially empty
	occupancy = 0;

	//bus has no current route, assigned stops or customers
	current_route.clear();
	assigned_stops.clear();
	assigned_customers.clear();

	//bus is idle
	idle = true;

	//set type of bus and initialize parameters accordingly
	type = param_type;
	init_by_type();
}

//reset a transporter as if starting a new simulation (mostly same as above)
void transporter::reset(ULL param_index, ULL param_location, ULL param_type)
{
	index = param_index;
	current_location = param_location;
	current_time = -1;

	node_of_last_stop = current_location;
	time_of_last_stop = -1;

	occupancy = 0;

	current_route.clear();
	assigned_stops.clear();
	assigned_customers.clear();

	idle = true;

	type = param_type;
	init_by_type();
}

//clear all containers
transporter::~transporter()
{
	current_route.clear();
	assigned_stops.clear();
	assigned_customers.clear();
}

//execute next event for the transporter (arrive at next node on route)
//returns the time of the next event (or -1 number if none)
double transporter::execute_event(double time, traffic_network& n, measurement_collector& m, ULL& total_serviced_requests, bool do_measurement)
{
	//make sure nothing broke in the simulation and we are actually on the route we are supposed to be on at the right time
	assert( !current_route.empty());
	assert( current_route.front().first == current_location);
	assert( current_route.front().second == time );
	assert( current_time == time );

	current_route.pop_front();

	//if stop event (pickup, dropoff, other) --> execute  [arriving at a node where something happens, not just passing through]
	if(current_route.empty())
	{
		//process stop from list of assigned stops
		assert(!assigned_stops.empty());
		stop current_stop = (*assigned_stops.begin());
		assigned_stops.erase(assigned_stops.begin());

		assert(current_location == current_stop.node_index);

		if( !is_idle() )	//measure only trips to pickup/delivery stops for requests, not other idle trips (e.g. to balance position or to depot)
		{
			m.measure_trip( std::make_pair(node_of_last_stop, time_of_last_stop), std::make_pair(current_location, current_time) );
			node_of_last_stop = current_location;
			time_of_last_stop = current_time;
		}

		//pickup event
		if( current_stop.is_pickup )
		{
			//pickup customer
			++occupancy;
			current_stop.c_it->set_pickup_time(current_time);

			assert(capacity < 0 || occupancy <= capacity);

		}else if( current_stop.is_dropoff )	//dropoff event
		{
			//drop off customer
			--occupancy;
			current_stop.c_it->set_dropoff_time(current_time);

			assert(occupancy >= 0);

			//measure the request
			++total_serviced_requests;
			if(do_measurement)
				m.measure_request( *(current_stop.c_it), *this );

			//customer is served and deleted from the system
			assigned_customers.erase( current_stop.c_it );
		}

		//if further stops planned, start driving there
		if(!assigned_stops.empty())
		{
			return( new_route( n.find_shortest_path(current_location, assigned_stops.begin()->node_index, current_time, velocity) ) );
		}else{//else become idle ( TODO: add drive back to random/nearest origin to balance asymmetric requests )

			idle = true;
			return(-1);	//no further event for this bus
		}

	}else{	//else, (no stop) the bus is just passing through a node on its route
		//update current location and time (arrival at next node on the route) and continue
		current_location = current_route.front().first;
		current_time = current_route.front().second;

		return(current_time);
	}
}

//update current route of the bus
double transporter::new_route( const std::deque< std::pair<ULL, double> > &param_new_route )
{
	//make sure the route is still current and goes to the next assigned stop
	assert( param_new_route.back().first == assigned_stops.begin()->node_index );

	//if not currently driving, simply start driving
	if(current_route.empty())
	{
		current_route = param_new_route;
		current_time = current_route.front().second;
		current_location = current_route.front().first;
		idle = false;
	}else{
		//else change the route, still have to finish driving on the current link
		current_route = param_new_route;
	}

	return(current_route.front().second);
}

//return best offer for the customer given the request and the currently best offer from all other buses

// this defines the dispatcher algorithm
// currently:
//		minimize arrival time (if multiple choices, secondary objective maximizes the pickup time, tertiary objective uses the bus with the larger occupancy)
//		under the constraint that other customers are not delayed
//		all conditions checked to within epsilon precision (to avoid problems due to addition along the route of a bus)
//

offer transporter::best_offer( ULL param_origin, ULL param_destination, double param_request_time, traffic_network &n, offer& current_best_offer )
{
	//request parameters
	ULL origin = param_origin;
	ULL destination = param_destination;
	double request_time = param_request_time;

	//variables for inserting the request in the scheduled route
	double temp_time_for_pickup = std::max(current_time, request_time);
	double temp_time_for_dropoff;
	double pickup_time;
	double dropoff_time;
	std::list< stop >::iterator temp_pickup_insertion;
	std::list< stop >::iterator temp_dropoff_insertion;

	bool pickup_is_possible = false;
	bool dropoff_is_possible = false;

	//current best offer
	offer best_offer(  current_best_offer.transporter_index, current_best_offer.best_transporter, current_best_offer.pickup_time, current_best_offer.pickup_insertion, current_best_offer.dropoff_time, current_best_offer.dropoff_insertion );

	ULL temp_location_for_pickup = current_location;
	ULL temp_location = current_location;
	LL occupancy_before_pickup = occupancy;
	LL occupancy_after_pickup = occupancy;

	//special case if the bus is idle
	if(idle)
	{
		//compute possible pickup and dropoff times
		pickup_time = temp_time_for_pickup + n.get_network_distance(current_location, origin) / velocity;
		dropoff_time = pickup_time + n.get_network_distance(origin, destination) / velocity;

		//new stops would be inserted at the end of the scheduled stop (since none are planned, the bus is idle)
		temp_pickup_insertion = assigned_stops.end();
		temp_dropoff_insertion = assigned_stops.end();

		//if better offer, remember
		if( dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
		  )
		{
			best_offer.transporter_index = index;
			best_offer.best_transporter = this;
			best_offer.pickup_insertion = temp_pickup_insertion;
			best_offer.dropoff_insertion = temp_dropoff_insertion;
			best_offer.pickup_time = pickup_time;
			best_offer.dropoff_time = dropoff_time;
			best_offer.is_better_offer = true;
		}else{
			//sanity check just to make sure nothing weird is going on
			assert( current_location != origin || pickup_time <= best_offer.pickup_time );
		}

		return(best_offer);
	}

	//only do all the checking if there can be a better offer
	if( current_time + n.get_network_distance(current_location, origin) / velocity + n.get_network_distance(origin, destination) / velocity < best_offer.dropoff_time + MACRO_EPSILON )
	{
		temp_dropoff_insertion = assigned_stops.end();

		//iterate over all possible insertions of pickup and delivery into the scheduled route of the bus
		for(temp_pickup_insertion = assigned_stops.begin(); temp_pickup_insertion != assigned_stops.end(); ++temp_pickup_insertion)
		{
			pickup_time = temp_time_for_pickup + n.get_network_distance(temp_location_for_pickup, origin) / velocity;	//time of pickup
			temp_time_for_dropoff = temp_time_for_pickup;
			pickup_is_possible = true;

			//check if transporter capacity allows for pickup
			if(capacity > 0 && occupancy_before_pickup >= capacity)
				pickup_is_possible = false;

			//check all if not delaying other requests
			if( n.get_network_distance(temp_location_for_pickup, origin) / velocity + n.get_network_distance(origin, temp_pickup_insertion->node_index) / velocity > n.get_network_distance(temp_location_for_pickup, temp_pickup_insertion->node_index ) / velocity + MACRO_EPSILON )
			{
				pickup_is_possible = false;
			}

			if( pickup_is_possible )
			{
				//track the number of people on the bus
				occupancy_after_pickup = occupancy_before_pickup + 1;
				temp_location = temp_location_for_pickup;
				for(temp_dropoff_insertion = temp_pickup_insertion; temp_dropoff_insertion != assigned_stops.end(); ++temp_dropoff_insertion)
				{
					//if drop off immediately after pickup, before going to the next scheduled stop
					if( temp_dropoff_insertion == temp_pickup_insertion )
					{
						dropoff_is_possible = true;
						dropoff_time = pickup_time + n.get_network_distance(origin, destination) / velocity;
						if( n.get_network_distance(temp_location, origin) / velocity + n.get_network_distance(origin, destination) / velocity + n.get_network_distance(destination, temp_dropoff_insertion->node_index ) / velocity > n.get_network_distance(temp_location, temp_dropoff_insertion->node_index ) / velocity + MACRO_EPSILON )
							dropoff_is_possible = false;

						//if the drop off is actually possible
						if( dropoff_is_possible )
						{
							//if this is a better dropoff
							if( dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
							  )
							{
								best_offer.transporter_index = index;
								best_offer.best_transporter = this;
								best_offer.pickup_insertion = temp_pickup_insertion;
								best_offer.dropoff_insertion = temp_dropoff_insertion;
								best_offer.pickup_time = pickup_time;
								best_offer.dropoff_time = dropoff_time;
								best_offer.is_better_offer = true;
							}
						}

					}else{	//if drop off somewhere on route
						dropoff_is_possible = true;
						dropoff_time = temp_time_for_dropoff + n.get_network_distance(temp_location, destination) / velocity;

						if( n.get_network_distance(temp_location, destination) / velocity + n.get_network_distance(destination, temp_dropoff_insertion->node_index ) / velocity > n.get_network_distance(temp_location, temp_dropoff_insertion->node_index ) / velocity + MACRO_EPSILON )
							dropoff_is_possible = false;

						//if the drop off is actually possible
						if( dropoff_is_possible )
						{
							//if this is a better dropoff
							if(	dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
							  )
							{
								best_offer.transporter_index = index;
								best_offer.best_transporter = this;
								best_offer.pickup_insertion = temp_pickup_insertion;
								best_offer.dropoff_insertion = temp_dropoff_insertion;
								best_offer.pickup_time = pickup_time;
								best_offer.dropoff_time = dropoff_time;
								best_offer.is_better_offer = true;
							}
						}
					}

					//advance location, occupancy etc. to check the next stop for dropoff
					temp_time_for_dropoff += n.get_network_distance(temp_location, temp_dropoff_insertion->node_index ) / velocity;
					temp_location = temp_dropoff_insertion->node_index;
					if(temp_dropoff_insertion->is_pickup)
						++occupancy_after_pickup;
					else if(temp_dropoff_insertion->is_dropoff)
						--occupancy_after_pickup;

					//if there cannot be a better offer from this dropoff forward, stop
					if(temp_time_for_dropoff + n.get_network_distance(temp_location, destination) / velocity > best_offer.dropoff_time + MACRO_EPSILON)
						break;
					//if the customer cannot be in the bus due to limited capacity, stop
					if(capacity >= 0 && occupancy_after_pickup > capacity)
						break;
				}

				//special case: drop off after all other stuff (pick up before)
				//no need to check delay, since no customer is delayed by drop off
				//if temp_dropoff_insertion is not at the end, the iteration stopped somewhere, because this dropoff is not possible or cannot be better
				if( temp_dropoff_insertion == assigned_stops.end() )
				{
					dropoff_time = temp_time_for_dropoff + n.get_network_distance(temp_location, destination) / velocity;

					//if the drop off at the end is a better offer
					if(dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
						( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
						( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
						( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
					  )
					{
						best_offer.transporter_index = index;
						best_offer.best_transporter = this;
						best_offer.pickup_insertion = temp_pickup_insertion;
						best_offer.dropoff_insertion = assigned_stops.end();
						best_offer.pickup_time = pickup_time;
						best_offer.dropoff_time = dropoff_time;
						best_offer.is_better_offer = true;
					}
				}
			}

			//advance location, occupancy etc. to check the next stop for pickup
			temp_time_for_pickup += n.get_network_distance(temp_location_for_pickup, temp_pickup_insertion->node_index ) / velocity;
			temp_location_for_pickup = temp_pickup_insertion->node_index;
			if(temp_pickup_insertion->is_pickup)
				++occupancy_before_pickup;
			else if(temp_pickup_insertion->is_dropoff)
				--occupancy_before_pickup;

			//make sure nothing broke
			assert(occupancy_before_pickup >= 0 && (capacity < 0 || occupancy_before_pickup <= capacity) );

			//if there cannot be a better from this pickup forward, stop
			if(temp_time_for_pickup + n.get_network_distance(temp_location_for_pickup, origin) / velocity + n.get_network_distance(origin, destination) / velocity > best_offer.dropoff_time + MACRO_EPSILON)
				break;
		}

		//special case: drop off after all other stuff (pick up before)
		//no need to check delay, since no customer is delayed
		//if temp_pickup_insertion is not at the end, the iteration stopped somewhere, because this pickup is not possible or cannot be better
		pickup_time = temp_time_for_pickup + n.get_network_distance(temp_location_for_pickup, origin) / velocity;
		if( temp_pickup_insertion == assigned_stops.end() )
		{
			dropoff_time = pickup_time + n.get_network_distance(origin, destination) / velocity;

			if(dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
			  )
			{
				best_offer.transporter_index = index;
				best_offer.best_transporter = this;
				best_offer.pickup_insertion = assigned_stops.end();
				best_offer.dropoff_insertion = assigned_stops.end();
				best_offer.pickup_time = pickup_time;
				best_offer.dropoff_time = dropoff_time;
				best_offer.is_better_offer = true;
			}
		}
	}

	//decide if best offer of this bus is better than the current best offer
	if(best_offer.dropoff_time < current_best_offer.dropoff_time - MACRO_EPSILON )
		best_offer.is_better_offer = true;
	else
		best_offer.is_better_offer = false;

	//return the best offer of this bus (contains whether it is a better offer)
	return(best_offer);
}


offer transporter::best_offer_unlimited( ULL param_origin, ULL param_destination, double param_request_time, traffic_network &n, offer& current_best_offer )
{
	//request parameters
	ULL origin = param_origin;
	ULL destination = param_destination;
	double request_time = param_request_time;

	//variables for inserting the request in the scheduled route
	double temp_time_for_pickup = std::max(current_time, request_time);
	double temp_time_for_dropoff;
	double pickup_time;
	double dropoff_time;
	std::list< stop >::iterator temp_pickup_insertion;
	std::list< stop >::iterator temp_dropoff_insertion;

	bool pickup_is_possible = false;
	bool dropoff_is_possible = false;

	//current best offer
	offer best_offer(  current_best_offer.transporter_index, current_best_offer.best_transporter, current_best_offer.pickup_time, current_best_offer.pickup_insertion, current_best_offer.dropoff_time, current_best_offer.dropoff_insertion );
	best_offer.is_better_offer = false;

	ULL temp_location_for_pickup = current_location;
	ULL temp_location = current_location;

	//special case if the bus is idle
	if(idle)
	{
		//compute possible pickup and dropoff times
		pickup_time = temp_time_for_pickup + n.get_network_distance(current_location, origin) / velocity;
		dropoff_time = pickup_time + n.get_network_distance(origin, destination) / velocity;

		//new stops would be inserted at the end of the scheduled stop (since none are planned, the bus is idle)
		temp_pickup_insertion = assigned_stops.end();
		temp_dropoff_insertion = assigned_stops.end();

		//if better offer, remember
		if( dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
		  )
		{
			best_offer.transporter_index = index;
			best_offer.best_transporter = this;
			best_offer.pickup_insertion = temp_pickup_insertion;
			best_offer.dropoff_insertion = temp_dropoff_insertion;
			best_offer.pickup_time = pickup_time;
			best_offer.dropoff_time = dropoff_time;
			best_offer.is_better_offer = true;
		}else{
			//sanity check just to make sure nothing weird is going on
			assert( current_location != origin || pickup_time <= best_offer.pickup_time );
		}

		return(best_offer);
	}

	//only do all the checking if there can be a better offer
	if( current_time + n.get_network_distance(current_location, origin) / velocity + n.get_network_distance(origin, destination) / velocity < best_offer.dropoff_time + MACRO_EPSILON )
	{
		temp_dropoff_insertion = assigned_stops.end();

		//iterate over all possible insertions of pickup and delivery into the scheduled route of the bus
		for(temp_pickup_insertion = assigned_stops.begin(); temp_pickup_insertion != assigned_stops.end(); ++temp_pickup_insertion)
		{
			pickup_time = temp_time_for_pickup + n.get_network_distance(temp_location_for_pickup, origin) / velocity;	//time of pickup
			temp_time_for_dropoff = temp_time_for_pickup;
			pickup_is_possible = true;

			//check all if not delaying other requests
			if( n.get_network_distance(temp_location_for_pickup, origin) / velocity + n.get_network_distance(origin, temp_pickup_insertion->node_index) / velocity > n.get_network_distance(temp_location_for_pickup, temp_pickup_insertion->node_index ) / velocity + MACRO_EPSILON )
			{
				pickup_is_possible = false;
			}

			if( pickup_is_possible )
			{
				temp_location = temp_location_for_pickup;
				for(temp_dropoff_insertion = temp_pickup_insertion; temp_dropoff_insertion != assigned_stops.end(); ++temp_dropoff_insertion)
				{
					//if drop off immediately after pickup, before going to the next scheduled stop
					if( temp_dropoff_insertion == temp_pickup_insertion )
					{
						dropoff_is_possible = true;
						dropoff_time = pickup_time + n.get_network_distance(origin, destination) / velocity;
						if( n.get_network_distance(temp_location, origin) / velocity + n.get_network_distance(origin, destination) / velocity + n.get_network_distance(destination, temp_dropoff_insertion->node_index ) / velocity > n.get_network_distance(temp_location, temp_dropoff_insertion->node_index ) / velocity + MACRO_EPSILON )
							dropoff_is_possible = false;

						//if the drop off is actually possible
						if( dropoff_is_possible )
						{
							//if this is a better dropoff
							if( dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
							  )
							{
								best_offer.transporter_index = index;
								best_offer.best_transporter = this;
								best_offer.pickup_insertion = temp_pickup_insertion;
								best_offer.dropoff_insertion = temp_dropoff_insertion;
								best_offer.pickup_time = pickup_time;
								best_offer.dropoff_time = dropoff_time;
								best_offer.is_better_offer = true;
							}
						}

					}else{	//if drop off somewhere on route
						dropoff_is_possible = true;
						dropoff_time = temp_time_for_dropoff + n.get_network_distance(temp_location, destination) / velocity;

						if( n.get_network_distance(temp_location, destination) / velocity + n.get_network_distance(destination, temp_dropoff_insertion->node_index ) / velocity > n.get_network_distance(temp_location, temp_dropoff_insertion->node_index ) / velocity + MACRO_EPSILON )
							dropoff_is_possible = false;

						//if the drop off is actually possible
						if( dropoff_is_possible )
						{
							//if this is a better dropoff
							if(	dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
								( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
							  )
							{
								best_offer.transporter_index = index;
								best_offer.best_transporter = this;
								best_offer.pickup_insertion = temp_pickup_insertion;
								best_offer.dropoff_insertion = temp_dropoff_insertion;
								best_offer.pickup_time = pickup_time;
								best_offer.dropoff_time = dropoff_time;
								best_offer.is_better_offer = true;
							}
						}
					}

					//advance location, occupancy etc. to check the next stop for dropoff
					temp_time_for_dropoff += n.get_network_distance(temp_location, temp_dropoff_insertion->node_index ) / velocity;
					temp_location = temp_dropoff_insertion->node_index;

					//if there cannot be a better offer from this dropoff forward, stop
					if(temp_time_for_dropoff + n.get_network_distance(temp_location, destination) / velocity > best_offer.dropoff_time + MACRO_EPSILON)
						break;
				}

				//special case: drop off after all other stuff (pick up before)
				//no need to check delay, since no customer is delayed by drop off
				//if temp_dropoff_insertion is not at the end, the iteration stopped somewhere, because this dropoff is not possible or cannot be better
				if( temp_dropoff_insertion == assigned_stops.end() )
				{
					dropoff_time = temp_time_for_dropoff + n.get_network_distance(temp_location, destination) / velocity;

					//if the drop off at the end is a better offer
					if(dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
						( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
						( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
						( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
					  )
					{
						best_offer.transporter_index = index;
						best_offer.best_transporter = this;
						best_offer.pickup_insertion = temp_pickup_insertion;
						best_offer.dropoff_insertion = assigned_stops.end();
						best_offer.pickup_time = pickup_time;
						best_offer.dropoff_time = dropoff_time;
						best_offer.is_better_offer = true;
					}
				}
			}

			//advance location, occupancy etc. to check the next stop for pickup
			temp_time_for_pickup += n.get_network_distance(temp_location_for_pickup, temp_pickup_insertion->node_index ) / velocity;
			temp_location_for_pickup = temp_pickup_insertion->node_index;

			//if there cannot be a better from this pickup forward, stop
			if(temp_time_for_pickup + n.get_network_distance(temp_location_for_pickup, origin) / velocity + n.get_network_distance(origin, destination) / velocity > best_offer.dropoff_time + MACRO_EPSILON)
				break;
		}

		//special case: drop off after all other stuff (pick up before)
		//no need to check delay, since no customer is delayed
		//if temp_pickup_insertion is not at the end, the iteration stopped somewhere, because this pickup is not possible or cannot be better
		pickup_time = temp_time_for_pickup + n.get_network_distance(temp_location_for_pickup, origin) / velocity;
		if( temp_pickup_insertion == assigned_stops.end() )
		{
			dropoff_time = pickup_time + n.get_network_distance(origin, destination) / velocity;

			if(dropoff_time < best_offer.dropoff_time - MACRO_EPSILON ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && pickup_time > best_offer.pickup_time + MACRO_EPSILON ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > best_offer.best_transporter->get_occupancy() ) ) ||
				( abs(dropoff_time - best_offer.dropoff_time) <= MACRO_EPSILON && abs(pickup_time - best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == best_offer.best_transporter->get_occupancy() ) )
			  )
			{
				best_offer.transporter_index = index;
				best_offer.best_transporter = this;
				best_offer.pickup_insertion = assigned_stops.end();
				best_offer.dropoff_insertion = assigned_stops.end();
				best_offer.pickup_time = pickup_time;
				best_offer.dropoff_time = dropoff_time;
				best_offer.is_better_offer = true;
			}
		}
	}

	//decide if best offer of this bus is better than the current best offer
	if(best_offer.dropoff_time < current_best_offer.dropoff_time - MACRO_EPSILON ) ||
				( abs(best_offer.dropoff_time - current_best_offer.dropoff_time) <= MACRO_EPSILON && best_offer.pickup_time > current_best_offer.pickup_time + MACRO_EPSILON ) ||
				( abs(best_offer.dropoff_time - current_best_offer.dropoff_time) <= MACRO_EPSILON && abs(best_offer.pickup_time - current_best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy > current_best_offer.best_transporter->get_occupancy() ) ) ||
				( abs(best_offer.dropoff_time - current_best_offer.dropoff_time) <= MACRO_EPSILON && abs(best_offer.pickup_time - current_best_offer.pickup_time) <= MACRO_EPSILON && ( best_offer.best_transporter != NULL && occupancy == current_best_offer.best_transporter->get_occupancy() ) )
			  )
		best_offer.is_better_offer = true;
	else
		best_offer.is_better_offer = false;

	//return the best offer of this bus (contains whether it is a better offer)
	return(best_offer);
}


//assign a customer based on the request and the offer made
double transporter::assign_customer(double assignment_time, customer c, offer& o, traffic_network &n)
{
	//make sure its the correct transporter!
	assert( o.transporter_index == index );

	//assign the customer to the transporter
	assigned_customers.push_front(c);
	std::list<customer>::iterator c_it = assigned_customers.begin();

	//if the transporter was idle, count time between stops from now (instead of from arrival at the last stop)
	if(idle)
	{
		node_of_last_stop = current_location;
		time_of_last_stop = assignment_time;
	}

	//if pickup right now (before the next stop), plan a new route to the new stop
	if( assigned_stops.empty() || o.pickup_insertion == assigned_stops.begin() )  //if empty, pickup insertion is end(), which is equal to begin() for empty lists
	{
		//insert the assigned stops
		assigned_stops.insert( o.pickup_insertion, stop(c.get_origin(), c_it, o.pickup_time, true, false, 0) );
		assigned_stops.insert( o.dropoff_insertion, stop(c.get_destination(), c_it, o.dropoff_time, false, true, 0) );

		//plan the new route

		if(current_route.empty())
		{
			//make sure bus was idle if there is no current route
			assert(idle);
			//plan route from the current location
			return( new_route( n.find_shortest_path(current_location, c.get_origin(), c.get_request_time(), velocity ) ) );
		}
		else
		{
			//plan route from the next node on the current route
			new_route( n.find_shortest_path(current_location, c.get_origin(), current_time, velocity) );
			//do NOT add a new event (bus is still driving to the next node on the route as in the old route)
			return(-1);
		}

	}else{	//else, simply insert the stops in the list of scheduled stops, the bus will continue its current route
		assert(!assigned_stops.empty());

		assigned_stops.insert( o.pickup_insertion, stop(c.get_origin(), c_it, o.pickup_time, true, false, 0) );
		assigned_stops.insert( o.dropoff_insertion, stop(c.get_destination(), c_it, o.dropoff_time, false, true, 0) );

		return(-1);
	}

}



//initialize capacity and velocity of transporters depending on their type
void transporter::init_by_type()
{
	//default behavior:
	//	velocity  = 1  (determines the timescale of the system, without loss of generality)
	//  capacity = -1  (negative values indicate infinite capacity)
	if( type == 0 )
	{
		velocity = 1;
		capacity = -1;
	}else{
		velocity = 1;
		capacity = type;
	}
}
