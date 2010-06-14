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

/** Implement a "Jogger," a task that will move an axis
 * forward by integrating the specified velocity and feeding the
 * resulting position to a feedback controller.
 * The jogger implements directional inhibits to prevent excessive motion.
 * @author DMAuslander, Nov. 17, 2009
 */
public abstract class Jogger extends TrjTask
{
    double s;  // Current position
    double sNeg, sPos; // Position limit for negative (positive) applied velocity
    double velJog;  // Jogging velocity
    boolean atMotionLimit;
    boolean eventFlag;
    protected boolean stopOnEvent;
    protected double tEvent = -1.0;  // Time at which an event occurred
    protected double posEvent = 0.0;  // Location at which event occurred

    public Jogger(String name, TrjSys sys,
            boolean taskActive, double dtNominal, double s0, double sNeg,
            double sPos, double velJog)
    {
        super(name, sys, 0, taskActive);  // Initial state (0) not used
        JoggerInit(dtNominal, s0, sNeg, sPos, velJog, false);
    }

    public Jogger(String name, TrjSys sys,
            boolean taskActive, double dtNominal, double s0, double sNeg,
            double sPos, double velJog, boolean stopOnEvent)
    {
        super(name, sys, 0, taskActive);  // Initial state (0) not used
        JoggerInit(dtNominal, s0, sNeg, sPos, velJog, stopOnEvent);
    }

    public void JoggerInit(double dtNominal, double s0, double sNeg,
            double sPos, double velJog, boolean stopOnEvent)
    {
        s = s0;  // Initial location
        this.dtNominal = dtNominal;
        this.sNeg = sNeg;
        this.sPos = sPos;
        this.velJog = velJog;
        atMotionLimit = false;
        eventFlag = false;
        stopOnEvent = this.stopOnEvent;
        tEvent = -1.0;  // Default value -- no event has occurred
    }

    public boolean RunTaskNow(TrjSys sys)
    {
        return CheckTime(sys.GetRunningTime());
    }     
    
    public boolean RunTask(TrjSys sys)
    {
        // No states in this task
        double t = sys.GetRunningTime();
        double sNew = s + velJog * dtNominal;  // Trial position

        // Don't advance the position if either the motion limit
        // has been hit or an event has occurred and stopOnEvent is TRUE.
        
        // Check against motion limits
        if(((velJog < 0.0) && (sNew >= sNeg)) ||
                ((velJog >= 0.0) && (sNew <= sPos)))
        {
            atMotionLimit = false;
        }
        else atMotionLimit = true;

        // Check for conditions for which it is OK to keep the profile moving
        // Otherwise, do nothing
        if(!(stopOnEvent && eventFlag) && !atMotionLimit)
        {
           s = sNew;  // OK to update
        }

        // Check for event
        if(EventCheck(t, s) && !eventFlag)
        {
            eventFlag = true;  // Once set, this can only
                // be cleared by a call to ClearEventFlag()
            tEvent = t;
            posEvent = s;
        }

        sToSetpoint(t, s);  // Send out the new setpoint
        return false;  // Done until next sample time
    }

    public void SetVelJog(double velJog)
    {
        this.velJog = velJog;
    }

    public void ClearEventFlag()
    {
        eventFlag = false;
        velJog = 0.0;  // To avoid unexpected motion
        tEvent = -1.0;
    }

    public boolean GetEventFlag()
    {
        return eventFlag;
    }
    
    public double GetEventTime()
    {
    	return tEvent;
    }
    
    public double GetEventLocation()
    {
    	return posEvent;
    }
    
    public boolean GetMotionLimitFlag()
    {
        return atMotionLimit;
    }
    
    /** This abstract function is used to send the setpoint to 
     * the relevant feedback control function.
     * @param t Time
     * @param s New setpoint
     */
    public abstract void sToSetpoint(double t, double s);
    
    /** This method is a dummy here, designed to be overridden if
     * event checking is used.
     * @param t Current time
     * @param s Current position
     * @return true if the event has been detected
     */
    public boolean EventCheck(double t, double s)
    {
        return false;  // Dummy, always returns false
    }
}