unset mouse
set multiplot layout 2,1 # This feature requires gnuplot v4.2 or later

# Constants for conversion to display units
RevPerRad = 1/(2.0 * pi)
RevPmPerRadPs = 60.0 * RevPerRad

file = 'TestData.txt'
set xlabel 'Time (sec)'


set ylabel 'Position (rev)'
set key bottom
set lmargin 12
plot file using 1:($2 * RevPerRad) title "Position0, rev" with lines,\
  file using 1:($7 * RevPerRad) title "Setpoint0, rev" with lines 

set ylabel 'Position (rev)'
set key bottom
set lmargin 12
plot file using 1:($9 * RevPerRad) title "Position1, rev" with lines,\
  file using 1:($14 * RevPerRad) title "Setpoint1, rev" with lines 


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
