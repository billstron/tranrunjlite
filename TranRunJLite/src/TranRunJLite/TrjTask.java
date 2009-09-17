/*
Copyright (c) 2009, Regents of the University of California
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

/** The task class. Tasks in this system all have equal priority.
 * TrjTask is abstract; the extending class must define RunTask()
 * which is where the specific task activities take place.
 * @author DMA
 */
public abstract class TrjTask {

    protected String name;
    protected int currentState;
    protected int nextState;
    protected int initialState;
    protected boolean trackState;
    protected ArrayList<String> stateNames = null;
    protected boolean runEntry;
    protected boolean taskActive;  // "true" for active
    protected int command;  // For a command that might be sent to the task
    // The actual commands are defined in user code
    TrjSys sys;  // The system that this task is part of
    double tNext, tPrev;  // If this task is sample-time based, these are
    // the times for the next and previous execution times
    protected double dtNominal, dtActual; // Desired and actual execution intervals
    boolean useNominalDT = true;
    boolean triggerMode = false;  // Runs only on a trigger rather than time
    boolean trigger;

    /** Constructor for TrjTask
     *
     * @param name the name of this task
     * @param sys the TrjSys of which this task is a part
     * @param initialState The state that will be executed first
     * @param taskActive If TRUE this task will be active
     * @param stateNames A list of the names of the states
     */
    public TrjTask(String name, TrjSys sys, int initialState,
            boolean taskActive) {
        this.name = name;
        this.sys = sys;
        this.initialState = initialState;
        this.taskActive = taskActive;
        this.stateNames = new ArrayList<String>();
        currentState = -1;  // So Entry will run initially
        nextState = initialState;
        trackState = false;  // Default is off
        command = -1;  // No command active
        sys.AddTask(this);  // Add this task to the list
    }

    /** Update the state information for this task. The main job
     * here is to figure out if the task should execute its ENTRY
     * section.
     */
    public void UpdateState() {
        // If there is a transition update; otherwise, do nothing
        if (nextState != -1) {
            int nNames = stateNames.size();
            if (trackState) {
                double t = sys.GetRunningTime();
                System.out.printf("t = %g, Task %s From ", t, name);
                if ((nNames == 0) || (currentState < 0)) {
                    System.out.printf("%d To ", currentState);
                } else {
                    System.out.printf("%s To ", stateNames.get(currentState));
                }
                if (nNames == 0) {
                    System.out.printf("%d\n", nextState);
                } else {
                    System.out.printf("%s\n", stateNames.get(nextState));
                }
            }
            currentState = nextState;
            nextState = -1;
            runEntry = true;
        } else {
            runEntry = false;
        }
    }

    /** Issue a command to a task.
     * Only one command can be active -- there is no task queue.
     * A new command issued before the old one is executed will erase the
     * old command. The command value -1 is used to indicate that no unexecuted
     * commands are available.
     * @param cmd The command. Must be a non-negative integer
     */
    public void SetCommand(int cmd) {
        command = cmd;
    }

    /** Get the command. A return of -1 means no command is available
     *
     * @return The command value (-1 indicates no command is active)
     */
    public int GetCommand() {
        int cmd = command;
        command = -1;
        return cmd;
    }

    /** Set the tracking state for this task. When state tracking is on
     * all state transitions will be recorded.
     * @param st Turn state tracking on (true) or off (false)
     */
    public void SetStateTracking(boolean st) {
        trackState = st;
    }

    /** Set the trigger. If 'true' and if the controller is in trigger mode,
     * this will cause the controller to run. The trigger is cleared once
     * the controller has run.
     * @param t Trigger value
     */
    public void SetTrigger(boolean t) {
        trigger = t;
    }

    /** Set the trigger mode. If set ('true') the controller will only run
     * when its trigger has been set by another task, usually a supervisor
     * type of task that determines setpoints for a feedback controller.
     * @param t Trigger mode ('true' to put controller into trigger mode)
     */
    public void SetTriggerMode(boolean t) {
        triggerMode = t;
    }

    /** Check to see if this is the correct time to run the controller
     * It will use either time or the trigger value depending on which
     * mode the controller is in.
     * @param t Current time
     * @return 'true' to run the controller now
     */
    public boolean CheckTime(double t) {
        boolean rv = false;  // Return value

        if (triggerMode) {
            if (trigger) {
                trigger = false;  // clear the trigger
                rv = true;
                dtActual = t - tPrev;
                tPrev = t;
            }
        } else if (t >= tNext) {
            tNext += dtNominal;  // Set up for next execution
            rv = true;  // Yes, execute now
            dtActual = t - tPrev;
            tPrev = t;
        }

        return rv;
    }

    /** Each task defines its own version of RunTask
     * This method is abstract
     * @param sys The TrjSys of which this task is a part
     * @return repeatTask; if "true" this task is repeated immediately
     */
    public abstract boolean RunTask(TrjSys sys);

    /** Each task uses this method to determine whether it wants to
     * run or not.
     * @param sys The TrjSys of which this task is a part
     * @return "true" to run this task now.
     */
    public abstract boolean RunTaskNow(TrjSys sys);
}
