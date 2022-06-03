Measurements for the full output represent (per column):

///PARAMETERS
1 - measurement time
2 - number of incoming requests during measurement time
3 - number of served requests during measurement time (including requests from before the start of measurement, exlcuding assigned requests not yet delivered)

4 - number of transporters
5 - request rate lambda
6 - normalized request rate x

7 - mean pickup distance (based on network and request pattern)
8 - mean drop off distance (based on network and request pattern) [identical to 7 same for symmetric networks]
9 - asymmetry in the request pattern (sum_i |P_pick(i) - P_drop(i)|) [= 0 for uniform requests]

10 - total velocity B * v 

//MEASUREMENTS (NUMBER OF MEASUREMENTS TAKEN, AVERAGE, STDDEV [note: NOT standard error of the mean!])
11/12/13 waiting time t_w
14/15/16 drive time t_d
17/18/19 delay time t_delay
20/21/22 fraction of delayed requests due to capacity constraints p_full
23/24/25 fraction of delayed trips

26/27/28 drive time between stops (segment time) t_s (l_s / v)
29/30/31 drive distance between stops (segment length) l_s

32/33/34 occupancy O
35/36/37 number of scheduled customers C
38/39/40 number of planned stops n
41/42/43 planned time horizon T_n

44/45/46 number of idle transporters 




Measurements for the time series output represent (per column):
1 - time since start of measurement
2 - absolute simulation time
3 - normalized request rate x
4 - average current occupancy O (per transporter)
5 - average current number of scheduled customers C (per transporter)
6 - fraction of currently idle transporters

