unset mouse
set multiplot layout 3,2 # This feature requires gnuplot v4.2 or later

file = 'TestData.txt'
#file = 'TestDataRobotRealTune1.txt'

set xlabel 'Time (sec)'

nColPerMotor = 8
id = 0
offset = id * nColPerMotor
labM(n) = sprintf("Actuation%d,  duty cycle", n)
set ylabel 'Actuation (duty cycle)'
set key bottom
set lmargin 12
plot file using 1:6+offset title labM(id) with lines

id = 1
offset = id * nColPerMotor
labM(n) = sprintf("Actuation%d,  duty cycle", n)
set ylabel 'Actuation (duty cycle)'
set key bottom
set lmargin 12
plot file using 1:6+offset title labM(id) with lines

id = 2
offset = id * nColPerMotor
labM(n) = sprintf("Actuation%d,  duty cycle", n)
set ylabel 'Actuation (duty cycle)'
set key bottom
set lmargin 12
plot file using 1:6+offset title labM(id) with lines

id = 3
offset = id * nColPerMotor
labM(n) = sprintf("Actuation%d,  duty cycle", n)
set ylabel 'Actuation (duty cycle)'
set key bottom
set lmargin 12
plot file using 1:6+offset title labM(id) with lines

id = 4
offset = id * nColPerMotor
labM(n) = sprintf("Actuation%d,  duty cycle", n)
set ylabel 'Actuation (duty cycle)'
set key bottom
set lmargin 12
plot file using 1:6+offset title labM(id) with lines

id = 5
offset = id * nColPerMotor
labM(n) = sprintf("Actuation%d,  duty cycle", n)
set ylabel 'Actuation (duty cycle)'
set key bottom
set lmargin 12
plot file using 1:6+offset title labM(id) with lines


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
