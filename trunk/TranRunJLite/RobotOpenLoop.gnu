# Plot results of motor simulation using MotorSim

set multiplot layout 2,1 # This feature requires gnuplot v4.2 or later
# Constants for conversion to display units
RevPerRad = 1/(2.0 * pi)
RevPmPerRadPs = 60.0 * RevPerRad

#file = 'TestData-RobotOpenLoop.txt'
file = 'TestData.txt'

set lmargin 12
set ylabel 'Link Velocity (deg/s)'
plot file using 1:($3) title "Measured Velocity, deg/s" with lines,\
  file using 1:($5) title "Estimated Velocity, deg/s" with lines


set ylabel 'Link Position (deg)'
#set key bottom
set lmargin 12
plot file using 1:($2) title "Position, deg" with lines
