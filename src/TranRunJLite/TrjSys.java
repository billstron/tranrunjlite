/*
Copyright (c) 2008, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
 * Neither the name of the University of California, Berkeley
nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package TranRunJLite;

import java.util.ArrayList;
import java.util.GregorianCalendar;

/** TranRunJLite TrjSys class
 * Implements a simple version of a task/state system. This
 * code is based on TranRunCpp, which implements the same model.
 * The TrjSys class is where a "system" is defined
 * @author DMAuslander, July 2, 2009
 */
public class TrjSys {

    ArrayList<TrjTask> taskList = new ArrayList<TrjTask>();
    TrjTime tm;
    double tRunning;
    double tRunning0;
    boolean stop = false;

    /** Constructor for class TrjSys
     * This class is abstract -- child class must define a RunTask()
     * @param t0 Initial time value
     */
    public TrjSys(double t0) {
        tRunning0 = t0;
        tRunning = 0.0;
        tm = new TrjTimeSim(t0);
        stop = false;
    }

    /** Constructor for class TrjSys
     *
     * @param tm Timing structure to use
     */
    public TrjSys(TrjTime tm) {
        //tRunning0 = t0;
        //tRunning = 0.0;
        this.tm = tm;
        stop = false;
    }

    /** Add a task to this system. There is no limit on
     * the number of tasks that can be added.
     * @param aThis the task to be added
     */
    public void AddTask(TrjTask aThis) {
        taskList.add(aThis);
    }

    /** Get the running time. This is the time since the
     * system was turned on in seconds.
     * @return time since system was turned on (seconds)
     */
    public double GetRunningTime() {
        return tm.getRunningTime();
        //return tRunning;  // used before there were timers
    }

    /** Set the running time to a specified time.
     * 
     * @param t
     */
    public void SetRunningTime(double t) {
        double tNow = tm.getRunningTime();
        tm.incrementRunningTime(t - tNow);
        //tRunning = t - tRunning0;  // used before there were timers
    }

    /** Increment the running time by the specified time.
     * 
     * @param dt
     */
    public void IncrementRunningTime(double dt) {
        //tRunning += dt;  // For simulated time
        tm.incrementRunningTime(dt);
    }

    /** Get a date and timefor the current time based on the time
     * structure.  
     * @return
     */
    public GregorianCalendar GetCalendar() {
        return tm.getCalendar();
    }

    /** Get a date and time for the specified run time based on the time
     * structure.
     * @param t
     * @return
     */
    public GregorianCalendar GetCalendar(double t) {
        return tm.getCalendar(t);
    }

    /** Causes system to stop when current task returns
     */
    public void SetStop() {
        stop = true;
    }

    /** Run all of the tasks. Tasks are run in order, unless a
     * task returns "true" in which case it is repeated immediately.
     * @return "stop" - signals to calling function that this system is done.
     */
    public boolean RunTasks() {
        for (TrjTask tsk : taskList) {
            // Run one scan for all of the tasks
            boolean runTask = true;
            // skip inactive tasks
            if (!tsk.taskActive) {
                runTask = false;
            }
            // skip if task doesn't want to run
            if (!tsk.RunTaskNow(this)) {
                runTask = false;
            }

            // run as many consecutive times as it wants.
            while (runTask) {
                // update the states.
                tsk.UpdateState();
                runTask = tsk.RunTask(this);
                // stop immediately if indicated.
                if (stop) {
                    return stop;
                }
            }
        }
        return stop;  // Indicate whether the system has stopped running
    }
}
