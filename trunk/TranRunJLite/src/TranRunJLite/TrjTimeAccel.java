/*
 * Copyright (c) 2009, Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *  * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *  * Neither the name of the University of California, Berkeley
 * nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package TranRunJLite;

import PerfTimerPkg.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

/**A time class for TranRunJLite that operates in accelerated realtime.  
 *
 * @author WJBurke
 */
public class TrjTimeAccel implements TrjTime {

    private long nsStart;
    private long msStart;
    private double accel;

    /** Construct the TrjTime object
     *
     * @param accel -- time scale factor (t_sim = t_real * accel)
     */
    public TrjTimeAccel(double accel) {
        String osName = System.getProperty("os.name");
        if(osName.equalsIgnoreCase("Linux")){
            this.nsStart = System.nanoTime();
            this.msStart = System.currentTimeMillis();
        } else{
            PerfTimer.InitPerfTimer();
            this.nsStart = (long) (PerfTimer.GetPerfTime() * 1e9);
            this.msStart = System.currentTimeMillis();
        }
        this.accel = accel;
    }

    /** Get the Gregorian Calendar for the current time.
     *
     * @return -- Current date/time
     */
    public GregorianCalendar getCalendar() {
        long msNow = System.currentTimeMillis();
        long msAccel = (long) ((double) (msNow - msStart) * accel);

        GregorianCalendar cal = new GregorianCalendar();
        cal.setTimeInMillis(msStart + msAccel);

        return cal;
    }

    /** Get the Gregorian Calendar associated with a specified runtime.
     *
     * @param t -- runtime of desired calendar
     * @return date/time of this time
     */
    public GregorianCalendar getCalendar(double t) {
        GregorianCalendar cal = new GregorianCalendar();
        cal.setTimeInMillis(msStart + (long) (t * 1000));
        return cal;
    }

    /** Get the current time in runtime
     *
     * @return time (s) of the runtime
     */
    public double getRunningTime() {
        long nsCurrent = 0;

        String osName = System.getProperty("os.name");

        if(osName.equalsIgnoreCase("Linux")){
            nsCurrent = System.nanoTime();
        } else{
            nsCurrent = (long) (PerfTimer.GetPerfTime() * 1e9);
        }
        return accel * (double) (nsCurrent - nsStart) / 1e9;
    }

    /** Does nothing, because the time is real time
     * 
     * @param dt -- makes no difference
     */
    public void incrementRunningTime(double dt) {
        // nothing to do
    }

    /** Reset the running time to the current time.
     *
     */
    public void resetRunningTime() {
        String osName = System.getProperty("os.name");
        if(osName.equalsIgnoreCase("Linux")){
            this.nsStart = System.nanoTime();
            this.msStart = System.currentTimeMillis();
        } else{
            this.nsStart = (long) (PerfTimer.GetPerfTime() * 1e9);
            this.msStart = System.currentTimeMillis();
        }
    }

    
    /** Test function
     *
     * @param args -- none!
     */
    public static void main(String args[]) {
        double accel = 5;
        TrjTimeAccel tm = new TrjTimeAccel(accel);

        double dt = 10;
        double tNext = 0;
        PrintWriter dataFile0 = null;
        try {
            FileWriter fW = new FileWriter("dataFile0.txt");
            dataFile0 = new PrintWriter(fW);
        } catch (IOException e) {
            System.out.println("IO Error " + e);
            System.exit(1);  // File error -- quit
        }

        final double tStop = 30;
        double tt0 = System.nanoTime();
        double t = 0;

        dataFile0.printf("start date: %02d/%02d/%4d %02d:%02d:%02d.%03d\n",
                (tm.getCalendar().get(Calendar.MONTH) + 1),
                tm.getCalendar().get(Calendar.DATE),
                tm.getCalendar().get(Calendar.YEAR),
                tm.getCalendar().get(Calendar.HOUR_OF_DAY),
                tm.getCalendar().get(Calendar.MINUTE),
                tm.getCalendar().get(Calendar.SECOND),
                tm.getCalendar().get(Calendar.MILLISECOND));
        dataFile0.printf("finish date: %02d/%02d/%4d %02d:%02d:%02d.%03d\n",
                (tm.getCalendar(tStop).get(Calendar.MONTH) + 1),
                tm.getCalendar(tStop).get(Calendar.DATE),
                tm.getCalendar(tStop).get(Calendar.YEAR),
                tm.getCalendar(tStop).get(Calendar.HOUR_OF_DAY),
                tm.getCalendar(tStop).get(Calendar.MINUTE),
                tm.getCalendar(tStop).get(Calendar.SECOND),
                tm.getCalendar(tStop).get(Calendar.MILLISECOND));
        
        while (t <= tStop) {
            t = tm.getRunningTime();
            double tt1 = System.nanoTime();
            if (t >= tNext) {
                dataFile0.printf("rt, time, date: %f, %f, %02d/%02d/%4d %02d:%02d:%02d.%03d\n",
                        (tt1 - tt0) / 1e9,
                        tm.getRunningTime(),
                        (tm.getCalendar().get(Calendar.MONTH) + 1),
                        tm.getCalendar().get(Calendar.DATE),
                        tm.getCalendar().get(Calendar.YEAR),
                        tm.getCalendar().get(Calendar.HOUR),
                        tm.getCalendar().get(Calendar.MINUTE),
                        tm.getCalendar().get(Calendar.SECOND),
                        tm.getCalendar().get(Calendar.MILLISECOND));
                tNext += dt;
            }
        }
        dataFile0.print("final time: " + tm.getRunningTime());
        dataFile0.close();
    }
}
