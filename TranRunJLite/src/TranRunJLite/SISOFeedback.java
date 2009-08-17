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
public abstract class SISOFeedback extends TrjTask
{
    double setpoint;
    double m;  // Controller output, manipulated (actuation) variable
    double mMin, mMax;  // Output limits
    double mOff;  // Value of m to use when controller is in Off state
    double y;  // Process variable
    double err;  // error
    boolean first;  // TRUE if this is the first pass
    boolean triggerMode;  // Runs only on a trigger rather than time
    boolean trigger;
    double dt;  // Sampling interval
    double tNext;  // Time for next execution
    public abstract double FindProcessValue();  // Used locally to get the process value
    public abstract void PutActuationValue(double val);  // for use by the control and to set the actuation

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
      int initialState, boolean taskActive)
    {
        super(name, sys, initialState, taskActive);
        this.dt = dt;
        this.mMin = mMin;
        this.mMax = mMax;
        this.mOff = mOff;
        this.triggerMode = triggerMode;
        trigger = false;
    }

    /** Get the value of the process value that the controller is using
     * @return The process value that the controller is using
     */
    public double GetProcessValue()  // For public access to controller values
    {
        return y;
    }

    /** Get the actuation value that the controller has computer
     * @return The most recently computed actuatio value
     */
    public double GetActuationValue()
    {
        return m;
    }

    /** Get the setpoint the controller is currently using
     * @return The setpoint
     */
    public double GetSetpoint()
    {
        return setpoint;
    }

    /** Get the error value the controller is currently using
     * @return The error
     */
    public double GetError()
    {
        return err;
    }

    /** Set the controller's setpoint
     * @param s The setpoint
     */
    public void SetSetpoint(double s)
    {
        setpoint = s;
    }

    /** Set the trigger. If 'true' and if the controller is in trigger mode,
     * this will cause the controller to run. The trigger is cleared once
     * the controller has run.
     * @param t Trigger value
     */
    public void SetTrigger(boolean t)
    {
        trigger = t;
    }

    /** Set the trigger mode. If set ('true') the controller will only run
     * when its trigger has been set by another task, usually a supervisor
     * type of task that determines setpoints for a feedback controller.
     * @param t Trigger mode ('true' to put controller into trigger mode)
     */
    public void SetTriggerMode(boolean t)
    {
        triggerMode = t;
    }

    /** Check to see if this is the correct time to run the controller
     * It will use either time or the trigger value depending on which
     * mode the controller is in.
     * @param t Current time
     * @return 'true' to runt he controller now
     */
    public boolean CheckTime(double t)
    {
        if(triggerMode)
        {
            if(trigger)
            {
                trigger = false;  // clear the trigger
                return true;
            }
        }
        else if(t >= tNext)
        {
            tNext += dt;  // Set up for next execution
            return true;  // Yes, execute now
        }

        return false;  // No, not time to execute yet
    }

    /** Limit the value of the actuation to stay between mMin and
     * mMax.
     */
    public void LimitActuation()
    {
        if(m > mMax)m = mMax;
        else if(m < mMin)m = mMin;
    }
}
