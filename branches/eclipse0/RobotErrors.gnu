unset mouse
set multiplot layout 3,2 # This feature requires gnuplot v4.2 or later

file = 'TestData.txt'
#file = 'TestData-RobotOpenLoop.txt'

set xlabel 'Time (sec)'

id = 0
offset = id * 7
labM(n) = sprintf("Error%d,  deg", n)
set ylabel 'Error (deg)'
set key bottom
set lmargin 12
plot file using 1:8+offset title labM(id) with lines

id = 1
offset = id * 7
labM(n) = sprintf("Error%d,  deg", n)
set key bottom
set lmargin 12
plot file using 1:8+offset title labM(id) with lines

id = 2
offset = id * 7
labM(n) = sprintf("Error%d,  deg", n)
set key bottom
set lmargin 12
plot file using 1:8+offset title labM(id) with lines

id = 3
offset = id * 7
labM(n) = sprintf("Error%d,  deg", n)
set key bottom
set lmargin 12
plot file using 1:8+offset title labM(id) with lines

id = 4
offset = id * 7
labM(n) = sprintf("Error%d,  deg", n)
set key bottom
set lmargin 12
plot file using 1:8+offset title labM(id) with lines

id = 5
offset = id * 7
labM(n) = sprintf("Error%d,  cm", n)
set ylabel 'Error (cm)'
set key bottom
set lmargin 12
plot file using 1:8+offset title labM(id) with lines


unset multiplot
#pause -1  # Use this if a 'dash' (-) cannot be added easily
     # to the command line
