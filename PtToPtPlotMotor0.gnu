unset mouse
set multiplot layout 2,2 title "Motor #0" # This feature requires gnuplot v4.2 or later

# Constants for conversion to display units
RevPerRad = 1/(2.0 * pi)
RevPmPerRadPs = 60.0 * RevPerRad

file = 'TestData.txt'
set xlabel 'Time (sec)'

set lmargin 12

set ylabel 'Actuation, duty cycle'
plot file using 1:6 title "Actuation, duty cycle" with lines

set lmargin 12
set ylabel 'Velocity (rpm)'
plot file using 1:($3 * RevPmPerRadPs) title "Measured Velocity, rpm" with lines,\
  file using 1:($5 * RevPmPerRadPs) title "Estimated Velocity, rpm" with lines

set ylabel 'Position0 (rev)'
set key bottom
set lmargin 12
plot file using 1:($2 * RevPerRad) title "Position, rev" with lines,\
  file using 1:($7 * RevPerRad) title "Setpoint, rev" with lines 

set lmargin 12
set ylabel 'Error (rev)'
plot file using 1:($8 * RevPerRad) title "Error, rev" with lines


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
