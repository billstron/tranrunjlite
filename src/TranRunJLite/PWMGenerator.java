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

/**
 *
 * @author William Burke <billstron@gmail.com>
 */
public abstract class PWMGenerator extends TrjTask {

    private double period, tSig;
    private double tStart = 0;
    private double tOffHead, tOffTail, tOnTotal;
    private double ratio;
    private boolean PwmTriggerMode;
    private boolean PwmTrigger;
    private double sig = 0.0;
    private int dir = 1;
    private double sigHigh, sigLow;
    private int mode;

    /** This must be defined in the sub-class.
     * 
     * @param pwr
     * @param dir
     */
    public abstract void PutActuationValue(double pwr, int dir);
    
    /** State definition
     */
    private final int LOWHEAD_STATE = 0;
    private final int HIGH_STATE = 1;
    private final int LOWTAIL_STATE = 2;
    /** Mode definition
     */
    public final int OFF_MODE = 0;
    public final int ON_MODE = 1;
    /** Direction definition
     */
    public final int FORWARD = 1;
    public final int BACKWARD = -1;

    public PWMGenerator(String name, TrjSys sys, double period, double sigLow,
            double sigHigh, double dt, boolean triggerMode,
            boolean taskActive) {
        super(name, sys, 0, taskActive);

        // add the state names to the list
        stateNames.add("Low State");
        stateNames.add("High State");

        this.sigHigh = sigHigh;
        this.sigLow = sigLow;
        this.dtNominal = dt;
        this.period = period;
        this.PwmTriggerMode = triggerMode;
        this.PwmTrigger = false;
        this.mode = ON_MODE;
    }

    /** Set the current duty ratio.  
     * 
     * @param value
     */
    public void setDutyRatio(double ratio) {
        //System.out.println("Set Duty Ratio: " + ratio);
        this.ratio = ratio;
        // set the saturation
        if (this.ratio > 1.0) {
            this.ratio = 1.0;
        }
        if (this.ratio < -1.0) {
            this.ratio = -1.0;
        }
    }

    /** Trigger the PWM.
     * 
     */
    public void trigger() {
        this.PwmTrigger = true;
    }

    /** Set the start time for the pulse.  
     * 
     * @param t
     */
    public void setStartTime(double t) {
        this.tStart = t;
    }

    /** Returns the current binary signal
     * 
     * @return
     */
    public double getSignal() {
        return this.sig;
    }

    /** Returns the current binary direction bit
     * 
     * @return
     */
    public double getDirectionBit() {
        return this.dir;
    }

    /** Returns the current duty ratio
     * 
     * @return
     */
    public double getDutyRatio() {
        return this.ratio;
    }

    /** Check to see if this task is ready to run
     * @param sys The system in which this task is embedded
     * @return "true" if this task is ready to run
     */
    public boolean RunTaskNow(TrjSys sys) {
        return CheckTime(sys.GetRunningTime());
    }

    /** Run the states.
     * In trigger mode, the next pulse only fires when the trigger has been set.
     * When it is set, any current pulse is aborted, and the new pulse begins
     * right away.
     * In non-trigger mode, the pulses fire continuously based on the timer.
     * @param sys
     * @return
     */
    public boolean RunTask(TrjSys sys) {
        // allows for rerunning the task
        boolean rerun = false;
        // parse the command
        int cmd = GetCommand();
        if (cmd == ON_MODE) {
            mode = ON_MODE;
        }
        if (cmd == OFF_MODE) {
            mode = OFF_MODE;
        }
        // calculate the low/high/low times based on the start time.
        //System.out.println("Duty Ratio: " + ratio);
        tOnTotal = Math.abs(ratio) * period;
        tOffHead = tStart;
        if (tOffHead >= (period - tOnTotal)) {
            tOffHead = period - tOnTotal;
        }
        tOffTail = period - tOnTotal - tOffHead;
        // in trigger mode, force new transition
        if (PwmTriggerMode && PwmTrigger) {
            currentState = LOWHEAD_STATE;
            runEntry = true;
            PwmTrigger = false;
        }
        // go into the states
        switch (currentState) {

            case LOWHEAD_STATE:  // no power output at the start
                if (runEntry) {
                    //System.out.println("Low Head State entry at " + sys.GetRunningTime());
                    //System.out.printf("%3.3f, %3.3f, %3.3f\n", tOffHead,
                    //        tOnTotal, tOffTail);
                    tSig = 0;  // reset the running time
                    runEntry = false;
                }
                tSig += dtNominal;  // update the running time
                sig = sigLow;  // set the signal
                PutActuationValue(sig, dir);
                // Compute the transition.
                nextState = -1;
                // transition when the off head time is up
                if (tSig >= tOffHead) {
                    nextState = HIGH_STATE;
                    // rerun if the timer was zero
                    if (tOffHead < dtNominal) {
                        rerun = true;
                    }
                }
                // then based on the mode
                if (mode == OFF_MODE) {
                    nextState = LOWTAIL_STATE;
                }
                break;

            case HIGH_STATE:  // power output
                if (runEntry) {
                    //System.out.println("High State entry at " + sys.GetRunningTime());
                    tSig = 0;  // reset the running time
                    runEntry = false;
                }
                tSig += dtNominal;  // update the running time
                // Determine the outputs.
                sig = sigHigh;  // set the signal
                // determine the direction bit
                if (ratio >= 0) {
                    dir = FORWARD;
                } else {
                    dir = BACKWARD;
                }
                PutActuationValue(sig, dir);
                // determine the transition.
                nextState = -1;
                if (tSig >= tOnTotal) {
                    nextState = LOWTAIL_STATE;
                    //System.out.println("tOnTotal: " + tOnTotal);
                    // rerun if the timer was zero
                    if (tOnTotal < dtNominal) {
                        rerun = true;
                        //System.out.println("here");
                        }
                }
                // then based on the mode
                if (mode == OFF_MODE) {
                    nextState = LOWTAIL_STATE;
                }
                break;

            case LOWTAIL_STATE:  // off at the tail of the period
                if (runEntry) {
                    //System.out.println("Low Tail State entry at " + sys.GetRunningTime());
                    tSig = 0;  // reset the running time
                    runEntry = false;
                }
                tSig += dtNominal;  // update the running time
                sig = sigLow;  // set the signal
                PutActuationValue(sig, dir);
                // Compute the transition.
                nextState = -1;
                // decide based on the trigger mode.
                if (PwmTriggerMode) {
                    // in trigger mode.
                    // don't transition
                    } else {
                    // in time based mode.
                    // transition when the off head time is up
                    //System.out.println("PWMGenerator.RunTask(): tSig = " + tSig + ", tOffTail = " + tOffTail);
                    if (tSig >= tOffTail) {
                        nextState = LOWHEAD_STATE;
                        // rerun if the timer was zero
                        if (tOffTail < dtNominal) {
                            rerun = true;
                        }
                    }
                }
                // then based on the regurlar mode
                if (mode == OFF_MODE) {
                    nextState = -1;
                }
                break;
        }

        //System.out.println("PWMGenerator.RunTask(): nextState = " + nextState);
        return rerun;
    }
}
