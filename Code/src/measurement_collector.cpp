#include "measurement_collector.h"

measurement_collector::measurement_collector(traffic_network& param_network) : network(param_network)
{

}

measurement_collector::~measurement_collector()
{
	//dtor
}

void measurement_collector::measure_request(customer& c, transporter& t)
{
	new_measurement(wait_time, c.get_pickup_time() - c.get_request_time());
	new_measurement(drive_time, c.get_dropoff_time() - c.get_pickup_time());
	new_measurement(delay_time, c.get_dropoff_time() - c.get_pickup_time()  -  network.get_network_distance(c.get_origin(), c.get_destination()) / t.get_velocity()  );
	if(c.get_delayed_due_to_capacity())
		new_measurement(delayed_due_to_capacity, 1);
	else
		new_measurement(delayed_due_to_capacity, 0);

	if( c.get_dropoff_time() - c.get_pickup_time() > network.get_network_distance(c.get_origin(), c.get_destination()) / t.get_velocity() + MACRO_EPSILON )
		new_measurement(fraction_of_delayed_trips, 1);
	else
		new_measurement(fraction_of_delayed_trips, 0);
}

void measurement_collector::measure_trip( std::pair<ULL, double> last_stop, std::pair<ULL, double> current_stop )
{
	new_measurement(drive_distance_between_stops, network.get_network_distance(last_stop.first, current_stop.first) );
	new_measurement(drive_time_between_stops, current_stop.second - last_stop.second );
}

//void measurement_collector::measure_stops(...)
//{
//	new_measurement(drive_time_between_stops, c.get_pickup_time() - c.get_request_time());
//	new_measurement(drive_distance_between_stops, c.get_pickup_time() - c.get_request_time());
//}

void measurement_collector::measure_system_status(std::vector<transporter>& transporter_list, double time)
{
	ULL idle_transporters = 0;
	for(transporter& t : transporter_list)
	{
		new_measurement(occupancy, t.get_occupancy() );
		new_measurement(scheduled_customers, t.get_number_of_scheduled_customers());
		new_measurement(number_of_planned_stops, t.get_number_of_planned_stops());
		new_measurement(planned_time_horizon, t.get_planned_time_horizon(time) );
		if(t.is_idle())
			++idle_transporters;
	}
	new_measurement(number_of_idle_transporters, idle_transporters );
}

void measurement_collector::reset()
{
	wait_time.reset();
	drive_time.reset();
	delay_time.reset();
	delayed_due_to_capacity.reset();
	fraction_of_delayed_trips.reset();

	drive_time_between_stops.reset();
	drive_distance_between_stops.reset();

	occupancy.reset();
	scheduled_customers.reset();
	number_of_planned_stops.reset();
	planned_time_horizon.reset();
	number_of_idle_transporters.reset();

}

void measurement_collector::print(std::ofstream& out)
{
	wait_time.print(out);
	drive_time.print(out);
	delay_time.print(out);
	delayed_due_to_capacity.print(out);
	fraction_of_delayed_trips.print(out);

	drive_time_between_stops.print(out);
	drive_distance_between_stops.print(out);

	occupancy.print(out);
	scheduled_customers.print(out);
	number_of_planned_stops.print(out);
	planned_time_horizon.print(out);
	number_of_idle_transporters.print(out);

	out << std::endl;
}

void measurement_collector::new_measurement(measure& m, long double val)
{
	m.n += 1;
	delta1 = val - m.av;
	m.av += delta1 / m.n;
	delta2 = val - m.av;
	m.stddev += delta1 * delta2;
}
