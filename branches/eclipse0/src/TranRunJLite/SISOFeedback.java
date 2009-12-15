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

/** Base class for single-input, single-output feedback control.
 * This class has all of the basic stuff except for the actual
 * algorithm.
 * @author D.M.Auslander, July 13, 2009
 */
public abstract class SISOFeedback extends TrjTask {

    protected double setpoint;
    protected double m;  // Controller output, manipulated (actuation) variable
    protected double mMin, mMax;  // Output limits
    protected double mOff;  // Value of m to use when controller is in Off state
    protected double y;  // Process variable
    protected double err;  // error
    protected boolean first;  // TRUE if this is the first pass
    protected double dt;  // Sampling interval

    public abstract double FindProcessValue();  // Used locally to get the process value

    public abstract void PutActuationValue(double val);  // for use by the control and to set the actuation
    // Commands (public so they're accessible to user-defined classes)
    public final int SISO_START_CONTROL = 0;
    public final int SISO_STOP_CONTROL = 1;

    /** Constuctor for the single-input, single output feedback control
     * base class. It must be extended with a class containing and
     * algorithm and then extended by the user so that input and output
     * can be determined.
     * @param name Task name
     * @param sys TrjSys in which this task is embedded
     * @param dt Controller interval
     * @param mMin Minimum allowed actuation value
     * @param mMax Maximum ...
     * @param mOff Actuation value to use when the controller is off
     * @param triggerMode 'true' if controller only runs directly after
     * some other task task runs (usually that is a Supervisory task)
     * @param initialState State that is entered when controller becomes active
     * @param taskActive 'true' for state to be active
     */
    public SISOFeedback(String name, TrjSys sys, double dt,
            double mMin, double mMax, double mOff, boolean triggerMode,
            int initialState, boolean taskActive) {
        super(name, sys, initialState, taskActive);
        this.dtNominal = dt;
        this.mMin = mMin;
        this.mMax = mMax;
        this.mOff = mOff;
        this.triggerMode = triggerMode;
        trigger = false;
        if (triggerMode) {
            useNominalDT = false;  // Use actual dt for control calcs
        }
        setpoint = 0.0;
    }

    /** Get the value of the process value that the controller is using
     * @return The process value that the controller is using
     */
    public double GetProcessValue() // For public access to controller values
    {
        return y;
    }

    /** Get the actuation value that the controller has computer
     * @return The most recently computed actuatio value
     */
    public double GetActuationValue() {
        return m;
    }

    /** Get the setpoint the controller is currently using
     * @return The setpoint
     */
    public double GetSetpoint() {
        return setpoint;
    }

    /** Get the error value the controller is currently using
     * @return The error
     */
    public double GetError() {
        return err;
    }

    /** Set the controller's setpoint
     * @param s The setpoint
     */
    public void SetSetpoint(double s) {
        setpoint = s;
    }

    /** Set the minimum saturaiton limit.
     *
     * @param m
     */
    public void setMinM(double m) {
        this.mMin = m;
    }

    /** Set the maximum saturation limit
     * 
     * @param m
     */
    public void setMaxM(double m) {
        this.mMax = m;
    }

    /** Limit the value of the actuation to stay between mMin and
     * mMax.
     */
    public void LimitActuation() {
        if (m > mMax) {
            m = mMax;
        } else if (m < mMin) {
            m = mMin;
        }
    }
}
