# Plot results of motor simulation using MotorSim

set multiplot layout 2,1 # This feature requires gnuplot v4.2 or later
#file = 'dataFile0.txt'
file = 'dataFile0-real.txt'

set lmargin 12
set ylabel 'Motor Velocity (rpm)'
plot file using 1:5 title "Motor Speed, rpm" with lines

set ylabel 'Motor Position (rev)'
#set key bottom
set lmargin 12
plot file using 1:2 title "Motor Position, rev" with lines

