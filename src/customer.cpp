#include "customer.h"

customer::customer(ULL param_origin, ULL param_destination, double param_time, traffic_network& n, transporter& t, offer& o, bool param_delayed_due_to_capacity) :
	origin(param_origin), destination(param_destination), request_time(param_time), offer_pickup_time(o.pickup_time), offer_dropoff_time(o.dropoff_time), pickup_time(-1), dropoff_time(-1), delayed_due_to_capacity(param_delayed_due_to_capacity)
{

}

customer::~customer()
{
	//dtor
}
