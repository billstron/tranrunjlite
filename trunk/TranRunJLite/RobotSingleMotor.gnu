unset mouse
set multiplot layout 2,2 # This feature requires gnuplot v4.2 or later

# Constants for conversion to display units
file = 'TestData.txt'
set xlabel 'Time (sec)'

id = 5
offset = id * 7

set lmargin 12

labA(n) = sprintf("Actuation%d,  duty cycle", n)

set ylabel 'Actuation, duty cycle'
plot file using 1:6+offset title labA(id) with lines

labA(n) = sprintf("Measured Velocity%d,  deg/s", n)
labB(n) = sprintf("Estimated Velocity%d,  deg/s", n)
set lmargin 12
set ylabel 'Velocity (deg/s)'
plot file using 1:3+offset title labA(id) with lines,\
  file using 1:5+offset title labB(id) with lines

labA(n) = sprintf("Position%d,  deg", n)
labB(n) = sprintf("Setpoint%d,  deg", n)
set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:2+offset title labA(id) with lines,\
  file using 1:7+offset title labB(id) with lines 

labA(n) = sprintf("Error%d,  deg", n)
set lmargin 12
set ylabel 'Error (deg)'
plot file using 1:8+offset title labA(id) with lines


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
