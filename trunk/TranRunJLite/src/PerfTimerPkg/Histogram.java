/*
 * Histogram.java
 * 
 * Created on Dec 30, 2008, 10:28:47 AM
 * 
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

package PerfTimerPkg;

import java.io.*; 

/**
 *
 * @author User
 */
public class Histogram 
{
    protected int nBins;
    protected int type;  // 0: linear; 1: multiplicative
    protected double minVal, maxVal;  // Extent of the histogram
    protected double ck;  // Additive or multiplicative factor for bins
    protected int [] bins;
    protected double [] binEdgeVals;
    protected int nBelow,  nAbove;
    protected double highVal, lowVal, avgVal, sumVals;
    protected int nVals;
    protected boolean first;

    public Histogram(int type, int nBins, double minVal, double ck)
    {
        this.type = type;
        this.nBins = nBins;
        this.minVal = minVal;
        this.ck = ck;
        
        bins = new int[nBins];
        binEdgeVals = new double[nBins];

        if(type == 0)
        {
            // Linear histogram
            double db = ck;
            for(int i = 0; i < nBins; i++)binEdgeVals[i] = minVal + (i + 1) * db;
            maxVal = binEdgeVals[nBins - 1];
        }
        else if(type == 1)
        {
            // Multiplicative histogram
            if(minVal <= 0.0)
            {
                System.out.printf("minVal can't be <= 0.0 for multiplicative mode\n");
                System.exit(1);
            }
            double kb = ck;
            binEdgeVals[0] = kb * minVal;
            for(int i = 1; i < nBins; i++)binEdgeVals[i] = kb * binEdgeVals[i - 1];
            maxVal = binEdgeVals[nBins - 1];
        }
        else
        {
            System.out.printf("<Histogram constructor> Unknown histogram type -- %d\n", type);
            System.exit(1);
        }
        
        for(int i = 0; i < nBins; i++)bins[i] = 0;
        nBelow = 0;
        nAbove = 0;
        avgVal = 0.0;
        sumVals = 0.0;
        nVals = 0;
        first = true;
    }
    
    public void HistValue(double v)
    {
        // Add a new value to the histogram
        if(first)
        {
            first = false;
            highVal = v;
            lowVal = v;
        }
        else
        {
            if(v > highVal)highVal = v;
            else if(v < lowVal)lowVal = v;
        }
        nVals++;
        sumVals += v;
        avgVal = sumVals / nVals;
        
        if(v < minVal)nBelow++;  // Check above or below limits first
        else if(v >= maxVal)nAbove++;
        else
        {
            // Normal case -- inside the histogram limits
            for(int i = 0; i < nBins; i++)
            {
                if(v < binEdgeVals[i])
                {
                    bins[i]++;
                    break;  // Done looking
                }
            }
        }
    }
    
    public void WriteHistogram(String fileName)
    {
        if(fileName == null)
        {
            // Write data to the screen
            System.out.printf("  >=           <         Number\n");
            System.out.printf("-infinity   %g  %d\n", minVal, nBelow);
            System.out.printf("%g   %g     %d\n", minVal, 
                    binEdgeVals[0], bins[0]);
            for(int i = 1; i < nBins; i++)
            {
                System.out.printf("%g   %g     %d\n", binEdgeVals[i - 1], 
                    binEdgeVals[i], bins[i]);
            }
            System.out.printf("%g   +infinity    %d\n", maxVal, nAbove);
            System.out.printf("lowVal %g, avgVal %g, highVal %g \n", lowVal, avgVal, highVal);
        }
        else
        {
            // Write to a file
            try
            {
                PrintWriter outFile = new PrintWriter(new FileWriter(fileName));
                // Write data to the screen
                outFile.printf("  >=           <         Number\n");
                outFile.printf("-infinity   %g  %d\n", minVal, nBelow);
                outFile.printf("%g   %g     %d\n", minVal, 
                        binEdgeVals[0], bins[0]);
                for(int i = 1; i < (nBins - 1); i++)
                {
                    outFile.printf("%g   %g     %d\n", binEdgeVals[i], 
                        binEdgeVals[i + 1], bins[i]);
                }
                outFile.printf("%g   +infinity    %d\n", maxVal, nAbove);
                outFile.printf("lowVal %g, avgVal %g, highVal %g \n", lowVal, avgVal, highVal);
                outFile.close();
            }
            catch(IOException e)
            {
                System.out.println("<WriteHistogram> IO Error " + e);
                System.exit(1);
            }                                         
        }
    }
    public static void main(String args[]){
        // Create a histogram
        Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);

        // decide on the OS
        boolean linux = false;
        String osName = System.getProperty("os.name");
        if(osName.equalsIgnoreCase("Linux")){
            linux = true;
            
        } 
        
        // initialize the timer
        double t0 = 0;
        if(linux)
        {
            t0 = (double) ((double)System.nanoTime() / 1e9);
        }
        else
        {
            PerfTimer.InitPerfTimer();
            t0 = PerfTimer.GetPerfTime();
        }
        double t = 0;
        double tf = 5.0;
        int nLoops = 0;
        double tLowest = 1.e20;
        double tHighest = 0.0;
        double tLast = t;
        boolean first = true;
        System.out.println(t0);

        while(t <= tf)
        {
            if(linux)
            {
                t = (double) ((double)System.nanoTime() / 1e9) - t0;
            }
            else
            {
                t = PerfTimer.GetPerfTime();
            }
            nLoops++;
            double dt = t - tLast;
            tLast = t;
            if(first)first = false;
            else h.HistValue(dt);  // Ignore the first pass
        }

        h.WriteHistogram(null);
        h.WriteHistogram("hist.txt");
    }

}
