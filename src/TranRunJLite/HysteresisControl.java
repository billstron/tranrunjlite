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

/** Hysteresis Controller for single-input, single-output systems.
 * This class is used as an extension of the SISOFeedback class
 * and implements a Hysteresid control algorithm.  
 * This class is also abstract -- to use it the user must 'extend'
 * this class and define FindProcessValue() and PutActuationValue(double val)
 * so the controller can get its input and know where to place its output.
 *
 * @author William Burke <billstron@gmail.com>
 */
public abstract class HysteresisControl extends SISOFeedback {

    double hystMax, ant;

    /** Super constructor for Hysteresis Control
     *
     * @param name
     * @param sys
     * @param dt -- how often to run.  
     * @param mMin -- Minimum actuation value
     * @param mMax -- Maximum actuation value
     * @param mOff -- Actuation value for off condition
     * @param initialState
     * @param taskActive
     * @param hystMax -- Single sided hysteresis bound
     * @param ant -- Anticipator value.  
     * @param triggerMode
     */
    public HysteresisControl(String name, TrjSys sys, double dt,
            double mMin, double mMax, double mOff,
            int initialState, boolean taskActive,
            double hystMax, double ant, boolean triggerMode) {

        super(name, sys, dt, mMin, mMax, mOff, triggerMode, initialState,
                taskActive);
        this.hystMax = Math.abs(hystMax);
        this.ant = Math.abs(ant);
        this.err = 0;
    }
    // States
    private final int HYST_OFF = 0;
    private final int HYST_ON = 1;

    /** Determines when to run the task
     * 
     * @param sys
     * @return
     */
    @Override
    public boolean RunTaskNow(TrjSys sys) {
        return CheckTime(sys.GetRunningTime());
    }

    /** Runs the hysteresis control.
     * 
     * @param sys
     * @return
     */
    public boolean RunTask(TrjSys sys) {

        // get the process value and compute the error
        y = FindProcessValue();
        err = setpoint - y;

        switch (currentState) {
            case HYST_OFF:  // Controller is off
                if (runEntry) {
                    runEntry = false;
                    PutActuationValue(mOff);
                }
                // Transition test
                nextState = -1;  // Default, stay in this state
                // Check for command to start control
                if (GetCommand() == SISO_START_CONTROL) {
                    nextState = HYST_ON;
                }
                break;

            case HYST_ON: // Run the control
                if (runEntry) {
                    runEntry = false;
                    first = true;
                }
                // Apply the anticipator.
                err -= Math.signum(m) * ant;

                // Check if heating or cooling
                if (m > mMin) {
                    // Currently in heating mode (naturally or forced)
                    // Wait until error passes the heating hysterisis
                    // point before switching to cooling
                    //System.out.println("error " + err);
                    if (err <= -hystMax) {
                        m = mMin;  // Switch to cooling
                    }
                    // Otherwise, leave actuation as-is
                } else {
                    // Currently in cooling mode (naturally or forced)
                    // Set actuation according to sign of error
                    //System.out.println("error " + err);
                    if (err > hystMax) {
                        m = mMax;
                    } else {
                        m = mMin;
                    }
                }
                LimitActuation();  // Keep 'm' in bounds
                PutActuationValue(m);  // send to the actuator.  
                first = false;  // reset the flag

                // Transition test
                nextState = -1;  // Default, stay in this state
                // Check for command to stop control
                if (GetCommand() == SISO_STOP_CONTROL) {
                    nextState = HYST_OFF;
                }
                break;

            default:  // shouldn't get here.  
                System.out.printf("<HysteresisControl> Unknown state (%d)\n",
                        currentState);
                System.exit(1);
        }
        return false;
    }
}
