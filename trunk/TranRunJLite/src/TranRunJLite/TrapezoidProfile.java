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

/** Implement a "trapezoid" profile (constant accel, constant derivative,
 *  constant decel)
 * This class extends the ProfileGenerator class so it only has
 * to provide the missing functions. It then is itself extended to
 * a user-defined class that talks to one or more controllers.
 * @author DMAuslander, July 30, 2009
 */
public abstract class TrapezoidProfile extends ProfileGenerator
{
    double dsdtCruise;  // Magnitude of value of dsdt during the constant
            // derivative portion of the profile
    double accel, decel; // Magnitudes of values of the acceleration
            // and deceleration (d2s/dt2) sections of the profile
    double kc;  // Factor for how much of profile must be during cruise
    double accelProf, decelProf; // Values used during actual profile
            // ie, with proper sign
    double dsdtProf;  // Value used during a specific profile for the
            // cruise (constant derivative) section of the profile --
            // include sign information
    double t1, t2;  // Switch times, accel to cruise, cruise to decel

    public TrapezoidProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtCruise,
            double accel, double decel)
    {
        super(name, sys, initialState, taskActive, dtNominal);
        this.dsdtCruise = Math.abs(dsdtCruise);
        this.accel = Math.abs(accel);
        this.decel = Math.abs(decel);
        kc = 0.6;  // This portion of the move distance must be during cruise
    }

    public void initProfile(double t)
    {
        if(sE > s0)
        {
            dsdtProf = dsdtCruise;
            accelProf = accel;
            decelProf = -decel;
        }
        else 
        {
            dsdtProf = -dsdtCruise;
            accelProf = -accel;
            decelProf = decel;
        }
        // Trial profile; may have to be modified if distance is too short
        double dist = Math.abs(sE - s0);
        double tAccel = dsdtCruise / accel;
        double distAccel = 0.5 * tAccel * dsdtCruise;
        double tDecel = dsdtCruise / decel;
        double distDecel = 0.5 * tDecel * dsdtCruise;
        double distCruise = dist - distAccel - distDecel;
        double tCruise = distCruise / dsdtCruise;

        if(distCruise < (kc * dist))
        {
            // Not enough of profile is during cruise-- reduce cruise velocity
            distCruise = kc * dist;
            double vCruise = Math.sqrt(2.0 * (dist - distCruise) *
                    accel * decel / (accel + decel));
            tCruise = distCruise / vCruise;
            tAccel = vCruise / accel;
            tDecel = vCruise / decel;
            if(sE > s0)dsdtProf = vCruise;
            else dsdtProf = -vCruise;
        }

        t0 = t;
        t1 = t0 + tAccel;
        t2 = t1 + tCruise;
        tE = t2 + tDecel;
        s = s0;
        dsdt = 0.0;
    }

    public void updateProfile(double t)
    {
        if(t <= t1)
        {
            // Accel
            dsdt = accelProf * (t - t0);
        }
        else if(t <= t2)
        {
            // Cruise
            dsdt = dsdtProf;
        }
        else if(t <= tE)
        {
            // decel
            dsdt = dsdtProf + (t - t2) * decelProf;
        }
        else
        {
            // end of profile
            dsdt = 0.0;
            profileDone = true;
        }
        s+= dsdt * dtNominal;
        sToSetpoint(t, s);  // Send value out to the controller
    }

    public void holdProfile(double t)
    {
        s = sE;
        dsdt = 0.0;
        sToSetpoint(t, s);
    }

    public abstract void sToSetpoint(double t, double s);
}
