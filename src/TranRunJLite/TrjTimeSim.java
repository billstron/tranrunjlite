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

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Calendar;
import java.util.GregorianCalendar;
import java.util.TimeZone;

/** A Simulation timer class that is based on TrjTime.  Time moves forward when 
 * the incrementRunningTime() function is called, and not before.  This allows
 * synchronized timing of objects in a simulation.  Be careful, though, because
 * the time will not move forward unless you call it!
 *
 * @author WJBurke
 */
public class TrjTimeSim implements TrjTime {

    long nsStart;
    long msStart;
    double t0;
    double t;

    /** Construct the TrjTimeSim timer with now as the start time/date.
     *
     * @param t0 -- initial runtime (s)
     */
    public TrjTimeSim(double t0) {
        this.nsStart = System.nanoTime();
        this.msStart = System.currentTimeMillis();
        this.t0 = t0;
        this.t = t0;
    }

    /** Construct the TrjTimeSim timer with a specified start time/date.
     *
     * @param cal -- start time (date/time)
     * @param t0 -- initial runtime (s)
     */
    public TrjTimeSim(GregorianCalendar cal, double t0) {
        this.nsStart = System.nanoTime();
        this.msStart = cal.getTimeInMillis();
        this.t0 = t0;
        this.t = t0;
    }

    /** Get a Gregorian Calendar for the current time
     *
     * @return date/time
     */
    public GregorianCalendar getCalendar() {
        GregorianCalendar cal = new GregorianCalendar();
        cal.setTimeInMillis(msStart + (long) (this.t * 1000));
        return cal;
    }

    /**Get a Gregorian Calendar for the specified time
     *
     * @param t -- desired time (s) for calendar
     * @return date/time
     */
    public GregorianCalendar getCalendar(double t) {
        GregorianCalendar cal = new GregorianCalendar();
        cal.setTimeInMillis(msStart + (long) (t * 1000));
        return cal;
    }

    /** Get the current runtime
     *
     * @return runtime (s)
     */
    public double getRunningTime() {
        return t;
    }

    /** Steps the current time forward by the amount specified
     *
     * @param dt amount (s) to step the time forward by
     */
    public void incrementRunningTime(double dt) {
        t += dt;
    }

    /** Reset the running time to the current time.
     *
     */
    public void resetRunningTime() {
        this.nsStart = System.nanoTime();
        this.msStart = System.currentTimeMillis();
        this.t = t0;
    }
    
    /** Test function
     *
     * @param args -- none
     */
    public static void main(String args[]) {
        GregorianCalendar cal = new GregorianCalendar(1977, Calendar.NOVEMBER,
                2, 1, 0, 0);
        cal.setTimeZone(TimeZone.getTimeZone("America/Los_Angeles"));
        double t0 = 0;
        double dt = 5;
        TrjTimeSim tm = new TrjTimeSim(cal, t0);

        PrintWriter dataFile0 = null;
        try {
            FileWriter fW = new FileWriter("dataFile0.txt");
            dataFile0 = new PrintWriter(fW);
        } catch (IOException e) {
            System.out.println("IO Error " + e);
            System.exit(1);  // File error -- quit
        }

        dataFile0.println("start time: " + tm.getRunningTime());
        final double tStop = 10;
        double t = 0;
        while (t <= tStop) {
            t = tm.getRunningTime();
            dataFile0.printf("time, date: %f, %02d/%02d/%4d %02d:%02d:%02d.%03d\n",
                    tm.getRunningTime(),
                    (tm.getCalendar().get(Calendar.MONTH) + 1),
                    tm.getCalendar().get(Calendar.DATE),
                    tm.getCalendar().get(Calendar.YEAR),
                    tm.getCalendar().get(Calendar.HOUR_OF_DAY),
                    tm.getCalendar().get(Calendar.MINUTE),
                    tm.getCalendar().get(Calendar.SECOND),
                    tm.getCalendar().get(Calendar.MILLISECOND));
            tm.incrementRunningTime(dt);
        }
        dataFile0.print("final time: " + tm.getRunningTime());
        dataFile0.close();
    }
}
