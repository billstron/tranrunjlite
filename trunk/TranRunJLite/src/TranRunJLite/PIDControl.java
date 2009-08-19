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

/** PID controller for single-input, single-output systems.
 * This class is used as an extension of the SISOFeedback class
 * and implements the PID algorithm.
 * This class is also abstract -- to use it the user must 'extend'
 * this class and define FindProcessValue() and PutActuationValue(double val)
 * so the controller can get its input and know where to place its output.
 * @author DMAuslander July 14, 2009
 */
public abstract class PIDControl extends SISOFeedback
{
    double kp, ki, kd;  // Control gains
    double integ0;  // Initial value of integrator
    double integ;  // Integrator value
    double prop;
    double deriv;
    double prevError;
    boolean useAntiWindup;

    /** PIDControl constructor. This class is abstract. User must extend
     * it to build an actual controller
     * @param name Name of this task
     * @param sys The TrjSys in which this task is embedded
     * @param dt Controller sampler interval
     * @param mMin Minimum actuation value that is allowed
     * @param mMax Maximum ...
     * @param mOff Actuation value to use when controller is 'Off'
     * @param initialState State that is entered when task is first activated
     * @param taskActive 'true' if task is active
     * @param kp Controller gains
     * @param ki ...
     * @param kd ...
     * @param integ0 Initial value of integrator
     * @param triggerMode 'true' if controller only runs directly after
     * some other task task runs (usually that is a Supervisory task)
     * @param useAntiWindup Impose an anti-windup regime
     */
    public PIDControl(String name, TrjSys sys, double dt,
        double mMin, double mMax, double mOff,
        int initialState, boolean taskActive,
        double kp, double ki, double kd, double integ0,
        boolean triggerMode, boolean useAntiWindup)
    {
        super(name, sys, dt, mMin, mMax, mOff, triggerMode, initialState,
                taskActive);
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.useAntiWindup = useAntiWindup;
        this.integ0 = integ0;
        first = true;
        integ = integ0;
        // Name the states
        stateNames.add("PID_OFF");  // 0
        stateNames.add("PID_ON");  // 1
    }

    // States
    final int PID_OFF = 0;
    final int PID_ON = 1;

    /** Check to see if this task is ready to run
     * @param sys The system in which this task is embedded
     * @return "true" if this task is ready to run
     */
    public boolean RunTaskNow(TrjSys sys)
    {
        return CheckTime(sys.GetRunningTime());
    }

    /** RunTask() - This is where the control algorithm goes
     * @param sys The TrjSys of which this task is part
     * @return repeatTask - 'true' to run the task again immediately (default
     * is 'false')
     */
    public boolean RunTask(TrjSys sys)
    {
        //double t = sys.GetRunningTime();
        if(useNominalDT)dt = dtNominal;
        else dt = dtActual;
        
        y = FindProcessValue();
        err = setpoint - y;

        switch(currentState)
        {
            case PID_OFF:  // Controller is off
                if(runEntry)
                {
                    runEntry = false;
                    PutActuationValue(mOff);
                }

                // Transition test
                nextState = -1;  // Default, stay in this state
                // Check for command to start control
                if(GetCommand() == SISO_START_CONTROL)nextState = PID_ON;
                break;

            case PID_ON:
                if(runEntry)
                {
                    runEntry = false;
                    first = true;
                    integ = integ0;
                }

                // Action section
                prop = kp * err;
                if(first)
                {
                    deriv = 0.0;  // Not enough information the first time
                    first = false;
                }
                else
                {
                    deriv = kd * (err - prevError) / dt;
                }
                integ += ki * err * dt;

                if(useAntiWindup)
                {
                    // Check for windup and limit integrator if needed
                    double mTrial = prop + deriv + integ;
                    if(mTrial > mMax)
                    {
                        integ = mMax - (deriv + prop);
                        if(integ < 0.0)integ = 0.0;
                    }
                    else if(mTrial < mMin)
                    {
                        integ = mMin - (deriv + prop);
                        if(integ > 0.0)integ = 0.0;
                    }
                }
                m = prop + integ + deriv;
                LimitActuation();  // Keep 'm' in bounds
                PutActuationValue(m);
                //printf("<PID>m %g prop %g integ %g deriv %g err %g\n",
                //    m, integ, prop, deriv, err);
                prevError = err;
                first = false;

                // Transition test
                nextState = -1;  // Default, stay in this state
                // Check for command to stop control
                if(GetCommand() == SISO_STOP_CONTROL)nextState = PID_OFF;
                break;

                default:
                    System.out.printf("<PIDControl> Unknown state (%d)\n",
                        currentState);
                    System.exit(1);
        }
        return false;
    }
}
