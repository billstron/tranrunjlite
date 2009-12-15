# Run with: >awk -f LatexTable.awk HistDataAutomatedReal.txt
BEGIN {OFS = " & "; ORS = "\\\\\n" }
{print $1, $2, $3}
