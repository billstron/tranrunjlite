unset mouse
set multiplot layout 3,2 # This feature requires gnuplot v4.2 or later

file = 'TestData.txt'
#file = 'TestData-RobotOpenLoop.txt'

set xlabel 'Time (sec)'
nColPerMotor = 8

id = 0
offset = id * nColPerMotor
labM(n) = sprintf("Measured Velocity%d,  deg/s", n)
labE(n) = sprintf("Estimated Velocity%d,  deg/s", n)
set ylabel 'Velocity (deg/s)'
set key bottom
set lmargin 12
plot file using 1:3+offset title labM(id) with lines,\
  file using 1:5+offset title labE(id) with lines 

id = 1
offset = id * nColPerMotor
labM(n) = sprintf("Measured Velocity%d,  deg/s", n)
labE(n) = sprintf("Estimated Velocity%d,  deg/s", n)
set ylabel 'Velocity (deg/s)'
set key bottom
set lmargin 12
plot file using 1:3+offset title labM(id) with lines,\
  file using 1:5+offset title labE(id) with lines 
  
id = 2
offset = id * nColPerMotor
labM(n) = sprintf("Measured Velocity%d,  deg/s", n)
labE(n) = sprintf("Estimated Velocity%d,  deg/s", n)
set ylabel 'Velocity (deg/s)'
set key bottom
set lmargin 12
plot file using 1:3+offset title labM(id) with lines,\
  file using 1:5+offset title labE(id) with lines 
  
id = 3
offset = id * nColPerMotor
labM(n) = sprintf("Measured Velocity%d,  deg/s", n)
labE(n) = sprintf("Estimated Velocity%d,  deg/s", n)
set ylabel 'Velocity (deg/s)'
set key bottom
set lmargin 12
plot file using 1:3+offset title labM(id) with lines,\
  file using 1:5+offset title labE(id) with lines 
  
id = 4
offset = id * nColPerMotor
labM(n) = sprintf("Measured Velocity%d,  deg/s", n)
labE(n) = sprintf("Estimated Velocity%d,  deg/s", n)
set ylabel 'Velocity (deg/s)'
set key bottom
set lmargin 12
plot file using 1:3+offset title labM(id) with lines,\
  file using 1:5+offset title labE(id) with lines 
  
id = 5
offset = id * nColPerMotor
labM(n) = sprintf("Measured Velocity%d,  cm/s", n)
labE(n) = sprintf("Estimated Velocity%d,  cm/s", n)
set ylabel 'Velocity (deg/s)'
set key bottom
set lmargin 12
plot file using 1:3+offset title labM(id) with lines,\
  file using 1:5+offset title labE(id) with lines 


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
