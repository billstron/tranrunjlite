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

/** Implement a "cosine" profile (S-shaped velocity profile with constant
 * velocity cruise section)
 * This class extends the ProfileGenerator class so it only has
 * to provide the missing functions. It then is itself extended to
 * a user-defined class that talks to one or more controllers.
 * @author DMAuslander, July 30, 2009
 */
public abstract class CosineProfile extends ProfileGenerator
{
    protected double dsdtCruise;  // Magnitude of value of dsdt during the constant
            // derivative portion of the profile
    protected double Vc;  // Value of cruise vel. used during current profile
    protected double accelMax, decelMax; // Magnitudes of values of the maximum acceleration
            // and deceleration (d2s/dt2)
    protected double kc;  // Factor for how much of profile must be during cruise
    protected double accelProf, decelProf; // Values used during actual profile
            // ie, with proper sign
    protected double dsdtProf;  // Value used during a specific profile for the
            // cruise (constant derivative) section of the profile --
            // include sign information
    protected double Ta, Td;  // Accel, decel times
    protected double tCruise;
    protected double t1, t2;  // Switch times, accel to cruise, cruise to decel
    protected double sign = 1.0;  // Direction of motion
    protected double dMove;  // Magnitude of distance to be moved
    protected double dAccel, dDecel, dCruise; // Magnitudes of distances
    protected boolean nullProfile = false; // For the case where sE = s0
    final double pi = Math.PI;

    public CosineProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtCruise,
            double accelMax, double decelMax)
    {
        super(name, sys, initialState, taskActive, dtNominal);
        this.dsdtCruise = Math.abs(dsdtCruise);
        this.accelMax = Math.abs(accelMax);
        this.decelMax = Math.abs(decelMax);
        kc = 0.5;  // This portion of the move distance must be during cruise
    }

    public void initProfile(double t)
    {
        nullProfile = false;
        t0 = t;
        Vc = dsdtCruise;
        if(sE == s0)
        {
            nullProfile = true;
            return;
        }
        dMove = Math.abs(sE - s0);
        Ta = (Vc / 2.0) * pi * Math.sin(pi / 2.0) / accelMax;
        Td = (Vc / 2.0) * pi * Math.sin(pi / 2.0) / decelMax;
        dAccel = (Vc/2.0)*(Ta - (Ta/pi)*Math.sin(pi));
        dDecel = (Vc/2.0)*(Td - (Td/pi)*Math.sin(pi));
        dCruise = dMove - dAccel - dDecel;
        tCruise = dCruise / Vc;
        if ((dCruise / dMove) < kc)
        {
            // Cruise section is too short, redo based on longer cruise time
            dCruise = kc * dMove;
            Vc = Math.sqrt(4 * accelMax * decelMax * dMove * kc / (pi * (accelMax + decelMax)));
            Ta = (Vc / 2) * pi * Math.sin(pi / 2) / accelMax;
            Td = (Vc / 2) * pi * Math.sin(pi / 2) / decelMax;
            dAccel = (Vc/2)*(Ta - (Ta/pi)*Math.sin(pi));
            dDecel = (Vc/2)*(Td - (Td/pi)*Math.sin(pi));
            double dC = dMove - dAccel - dDecel;
            dCruise = dC;
            tCruise = dCruise / Vc;
        }
        //System.out.printf("<InitProfile> Vc %g Ta %g Td %g dAccel %g dDecel %g dMove %g\n",
        //        Vc, Ta, Td, dAccel,dDecel, dMove);
        t1 = Ta + t0;
        t2 = t1 + tCruise;
        tE = t2 + Td;
        // Set up for moves in either direction
        if (sE < s0)sign = -1.0;
        else sign = 1.0;
    }

    public void updateProfile(double t)
    {
        double v, x, a;  // velocity, position, acceleration
        double tp = t - t0;

        if(nullProfile)
        {
            s = sE;
            dsdt = 0.0;
            sToSetpoint(t, sE);  // Send value out to the controller
            profileDone = true;
            return;
        }
        if(t <= t1)
        {
            // Accel
            v = sign * (Vc/2)*(1-Math.cos(pi * tp / Ta));
            x = sign * (Vc/2)*(tp - (Ta/pi)*Math.sin(pi * tp / Ta)) + s0;
            a = sign * (Vc/2) * (pi / Ta) * Math.sin(pi * tp / Ta);
            s = x;
            dsdt = v;
            //dsdt = accelProf * (t - t0);
        }
        else if(t <= t2)
        {
            // Cruise
            v = sign * Vc;
            x = s0 + sign * dAccel + v * (t - t1);
            a = 0.0;
            s = x;
            dsdt = v;
        }
        else if(t <= tE)
        {
            // decel
            double ttt = t - t2;  // Time in decel
            v = sign * (Vc/2)*(1 + Math.cos(pi * ttt / Td));
            x = sign * (Vc/2)*(ttt + (Td/pi)*Math.sin(pi * ttt / Td)) + s0 + sign * (dAccel + dCruise);
            a = sign * (Vc/2) * (pi / Td) * Math.sin(pi * ttt / Td);
            s = x;
            dsdt = v;
        }
        else
        {
            // end of profile
            dsdt = 0.0;
            profileDone = true;
        }
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
