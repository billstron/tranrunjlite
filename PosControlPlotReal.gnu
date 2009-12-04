unset mouse
set multiplot layout 2,2 # This feature requires gnuplot v4.2 or later

file = 'dataFile0.txt'
set xlabel 'Time (sec)'

set lmargin 12
set ylabel 'Actuation, duty cycle'
plot file using 1:6 title "Actuation, duty cycle" with lines

set lmargin 12
set ylabel 'Velocity (rpm)'
plot file using 1:5 title "EstimatedVelocity, rpm" with lines

set ylabel 'Position (rev)'
set key bottom
set lmargin 12
plot file using 1:2 title "Position, rev" with lines,\
  file using 1:7 title "Setpoint, rev" with lines 

set lmargin 12
set ylabel 'Error (rev)'
plot file using 1:8 title "Error, rev" with lines


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
