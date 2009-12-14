unset mouse
set multiplot layout 3,2 # This feature requires gnuplot v4.2 or later

file = 'TestData.txt'
#file = 'TestDataRobotRealTune0.txt'
set xlabel 'Time (sec)'


set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:2 title "Position0, deg" with lines,\
  file using 1:7 title "Setpoint0, deg" with lines 

set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:9 title "Position1, deg" with lines,\
  file using 1:14 title "Setpoint1, deg" with lines 

set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:16 title "Position2, deg" with lines,\
  file using 1:21 title "Setpoint2, deg" with lines 

set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:23 title "Position3, deg" with lines,\
  file using 1:28 title "Setpoint3, deg" with lines 

set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:30 title "Position4, deg" with lines,\
  file using 1:35 title "Setpoint4, deg" with lines 

set ylabel 'Position (cm)'
set key bottom
set lmargin 12
plot file using 1:37 title "Position5, cm" with lines,\
  file using 1:42 title "Setpoint5, cm" with lines 



unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
