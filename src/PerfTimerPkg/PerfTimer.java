/*
 * PerfTimer.java
 * 
 * Created on Dec 28, 2008, 9:50:17 PM
 * 
 * To change this template, choose Tools | Template Manager
 * and open the template in the editor.
 */

package PerfTimerPkg;

/**
 *
 * @author User
 */
public class PerfTimer 
{

    public PerfTimer() 
    {
    }

    static 
    { 
      System.loadLibrary("PerfTimerJNI"); 
    }  

    public static native void InitPerfTimer(); 
    public static native double GetPerfTime(); 

    /** This main() can be used to test the basic timer operation
     */ 
    public static void main(String[] args) 
    {
        // Create a histogram
        Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);
        
        InitPerfTimer();
        System.out.println(GetPerfTime());
        
        double tf = 5.0;
        int nLoops = 0;
        double t = GetPerfTime();
        double tLowest = 1.e20;
        double tHighest = 0.0;
        double tLast = t;
        boolean first = true;
        
        while(t <= tf)
        {
            t = GetPerfTime();
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
