reset

unset xtics
unset ytics

set term pdf color solid enhanced
set output 'FIG2_a_main.pdf'

plot [0:12000][0:20] 'min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_x_3_timeseries.dat' u 1:5 w l not, 'min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_x_6_timeseries.dat' u 1:5 w l not, 'min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_x_9_timeseries.dat' u 1:5 w l not

set output


set term pdf color solid enhanced
set output 'FIG2_a_inset.pdf'

plot [6000:12000][0:120] 'min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_x_3_timeseries.dat' u 1:5 w l not, 'min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_x_6_timeseries.dat' u 1:5 w l not, 'min_arrival_time__uniform_requests__torus_N_25_B_50_theta_8_x_9_timeseries.dat' u 1:5 w l not 

set output
reset