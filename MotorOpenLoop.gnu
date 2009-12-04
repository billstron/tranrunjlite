# Plot results of motor simulation using MotorSim

set multiplot layout 2,1 # This feature requires gnuplot v4.2 or later
# Constants for conversion to display units
RevPerRad = 1/(2.0 * pi)
RevPmPerRadPs = 60.0 * RevPerRad

#file = 'TestData-8v.txt'
file = 'TestData.txt'

set lmargin 12
set ylabel 'Motor Velocity (rpm)'
plot file using 1:($3 * RevPmPerRadPs) title "Measured Velocity, rpm" with lines,\
  file using 1:($5 * RevPmPerRadPs) title "Estimated Velocity, rpm" with lines


set ylabel 'Motor Position (rev)'
#set key bottom
set lmargin 12
plot file using 1:($2 * RevPerRad) title "Position, rev" with lines
