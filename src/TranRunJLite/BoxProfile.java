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

/** Implement a "box" profile (constant derivative)
 * This class extends the ProfileGenerator class so it only has
 * to provide the missing functions. It then is itself extended to
 * a user-defined class that talks to one or more controllers.
 * @author DMAuslander, July 28, 2009
 */
public abstract class BoxProfile extends ProfileGenerator
{
    double dsdtBox;  // Magnitude of value of dsdt during profile generation
    double dsdtProf;  // Value used during a specific profile

    public BoxProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtBox)
    {
        super(name, sys, initialState, taskActive, dtNominal);
        this.dsdtBox = dsdtBox;
    }

    public void initProfile(double t)
    {
        if(sE > s0)dsdtProf = dsdtBox;
        else dsdtProf = -dsdtBox;
        t0 = t;
        tE = t0 + Math.abs(s0 - sE) / dsdtBox;
        s = s0;
    }

    public void updateProfile(double t)
    {
        s += dsdtProf * dtNominal;
        // Check for end of profile
        if(sE > s0)
        {
            if(s >= sE)
            {
                profileDone = true;
                s = sE;
            }
        }
        else
        {
            if(s <= sE)
            {
                profileDone = true;
                s = sE;
            }
        }
        sToSetpoint(t, s);  // Send value out to the controller
    }

    public void holdProfile(double t)
    {
        s = sE;
        sToSetpoint(t, s);
    }

    public abstract void sToSetpoint(double t, double s);
}
