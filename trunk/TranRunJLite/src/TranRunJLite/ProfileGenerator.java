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

/** Profile Generator -- an  abstract class to support a variety of
 * profile generation modes. Profiles are used to produce a sequence of
 * values for a paramteric variable s and its derivative, dsdt. The
 * profiles are described by the shape of the derivative, "box" has
 * constant derivative, "trapezoid" has a constant slope followed by a
 * constant derivative, "cosine" has an S-shaped derivative curve.
 * @author DM Auslander, July 23, 2009
 */
public abstract class ProfileGenerator extends TrjTask
{
    double t0, tE;  // Time at the start and end of the profile
    double s0, sE;  // Profile variable value at the start and end of the profile
    double dsdt0, dsdtE; // Slope of profile variable at the start of the profile
    // New values are held until the new profile actually starts
    double s0new, sEnew;  // Profile variable value at the start and end of the profile
    double dsdt0new, dsdtEnew; // Slope of profile variable at the start of the profile
    double s, dsdt;  // Current value of profile variable and slope
    double sHold, dsdtHold;  // Values used during the HOLD state
    boolean profileDone = false;

    public ProfileGenerator(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal)
    {
        super(name, sys, initialState, taskActive);
        this.dtNominal = dtNominal;
        profileDone = false;

        // States -- names and constants
        stateNames.add("PROFILE_OFF");
        stateNames.add("PROFILE_RUN");
        stateNames.add("PROFILE_HOLD");
    }

    // States
    public static final int PROFILE_OFF = 0;
    public static final int PROFILE_RUN = 1;
    public static final int PROFILE_HOLD = 2;

    // Commands
    public static final int START_PROFILE = 0;
    public static final int STOP_PROFILE = 1;

    public boolean RunTaskNow(TrjSys sys)
    {
        return CheckTime(sys.GetRunningTime());
    }

    public boolean RunTask(TrjSys sys)
    {
        int cmd;
        double t = sys.GetRunningTime();
        nextState = -1;  // Default is stay in same state

        switch(currentState)
        {
            case PROFILE_OFF:
                // Nothing to do
                // Transitions - check for command
                cmd = GetCommand();
                if(cmd == -1)break;  // Stay in this state
                else if(cmd == START_PROFILE)
                {
                    nextState = PROFILE_RUN;  // Start a profile
                }
                break;

            case PROFILE_RUN:
                if(runEntry)
                {
                    // Initial activities for this state
                    // Copy new values to working values
                    s0 = s0new;
                    sE = sEnew;
                    dsdt0 = dsdt0new;
                    dsdtE = dsdtEnew;
                    initProfile(t); // Initialize the profile
                    profileDone = false;
                }
                // Action section - update the profile
                updateProfile(t);

                // Transition - check for profile done or for command
                cmd = GetCommand();
                // Ignore commands for the moment
                if(profileDone)
                {
                    sHold = sE;  // Profile outputs during hold state
                    dsdtHold = dsdtE;
                    nextState = PROFILE_HOLD;
                }
                break;

            case PROFILE_HOLD:
                // No entry section
                // Action - keep sending out final profile values
                // If dsdtHold is not zero, sHold will get integrated and
                // will not be constant during the hold state (profiles
                // usually end with zero velocity)
                holdProfile(t);

                // Transitions
                cmd = GetCommand();
                if(cmd == START_PROFILE)nextState = PROFILE_RUN;
                        // Start a new profile
                break;
                
                default:
                    System.out.printf("<ProfileGenerator> Unknown state (%d)\n",
                        currentState);
                    System.exit(1);

        }
        return false;
    }

    public void setNewProfile(double s0, double sE)
    {
        setNewProfile(s0, sE, 0.0, 0.0);
    }

    public void setNewProfile(double s0, double sE,
            double dsdt0, double dsdtE)
    {
        this.s0new = s0;
        this.sEnew = sE;
        this.dsdt0new = dsdt0;
        this.dsdtEnew = dsdtE;
    }

    public abstract void initProfile(double t);
    public abstract void updateProfile(double t);
    public abstract void holdProfile(double t);
}
