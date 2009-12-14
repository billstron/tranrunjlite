unset mouse
set multiplot layout 3,2 # This feature requires gnuplot v4.2 or later

file = 'TestData.txt'
#file = 'TestDataRobotRealTune0.txt'
set xlabel 'Time (sec)'

nColPerMotor = 8

id = 0
offset = id * nColPerMotor
set ylabel 'Position (deg)'
set key bottom
set lmargin 12
set y2range [-1.0:10.0]
set ytics nomirror
plot file using 1:2+offset title "Position0, deg" with lines axis x1y1,\
  file using 1:7+offset title "Setpoint0, deg" with lines, \
  file using 1:9+offset title "Switch0" with lines axis x1y2 
  

id = 1
offset = id * nColPerMotor
set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:2+offset title "Position1, deg" with lines,\
  file using 1:7+offset title "Setpoint1, deg" with lines,\
  file using 1:9+offset title "Switch1" with lines axis x1y2

id = 2
offset = id * nColPerMotor
set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:2+offset title "Position2, deg" with lines,\
  file using 1:7+offset title "Setpoint2, deg" with lines,\
  file using 1:9+offset title "Switch2" with lines axis x1y2

id = 3
offset = id * nColPerMotor
set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:2+offset title "Position3, deg" with lines,\
  file using 1:7+offset title "Setpoint3, deg" with lines,\
  file using 1:9+offset title "Switch3" with lines axis x1y2

id = 4
offset = id * nColPerMotor
set ylabel 'Position (deg)'
set key bottom
set lmargin 12
plot file using 1:2+offset title "Position4, deg" with lines,\
  file using 1:7+offset title "Setpoint4, deg" with lines,\
  file using 1:9+offset title "Switch4" with lines axis x1y2

id = 5
offset = id * nColPerMotor
set ylabel 'Position (cm)'
set key bottom
set lmargin 12
plot file using 1:2+offset title "Position5, cm" with lines,\
  file using 1:7+offset title "Setpoint5, cm" with lines,\
  file using 1:9+offset title "Switch5" with lines axis x1y2



unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
