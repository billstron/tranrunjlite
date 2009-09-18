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

package TranRunJLiteSamples;

import java.util.ArrayList;
import TranRunJLite.*;
import visaio.*;

/** Collects information for a set of motors so that they can be
 * operated in a synchronized way (or, independently, if desired)
 * This version assumes that each motor has its own simulation
 * object. That is, that the motors are not dynamically connected. If
 * they are, a different structure would be needed.
 *
 * @author DMAuslander, Sept. 10, 09
 */
public class MultiMotor
{
    // ArrayLists are more convenient than plain arrays, but probably less
    // efficient.
    ArrayList<Motor> mtr = null;
    ArrayList<MotorSim> mSim = null;
    ArrayList<SISOFeedback> fb = null;
    ArrayList<ProfileGenerator> prof = null;
    TrjSys sys = null;
    boolean realMotors;

    public MultiMotor(TrjSys sys, ArrayList<Motor> mtr,
            ArrayList<MotorSim> mSim, ArrayList<SISOFeedback> fb,
            ArrayList<ProfileGenerator> prof, boolean realMotors)
    {
        this.sys = sys;
        this.mtr = mtr;
        this.mSim = mSim;
        this.fb = fb;
        this.prof = prof;
        this.realMotors = realMotors;
    }

    public void AddMotor(Motor m)
    {
        mtr.add(m);
    }

    public void AddMotorSim(MotorSim ms)
    {
        mSim.add(ms);
    }

    public void AddFeedback(SISOFeedback f)
    {
        fb.add(f);
    }

    public void AddProfileGenerator(ProfileGenerator p)
    {
        prof.add(p);
    }

    public int NumberOfMotors()
    {
        return mtr.size();
    }

    public void FillDataRow(double t, double [] data)
    {
        Motor m;
        int n = mtr.size();  // Number of motors

        for(int i = 0; i < n; i++)
        {
            m = mtr.get(i);
            // Connect motor data to simulation or real motor
            data[0] = t;
            if(realMotors)
            {
/*                                     m0.getEngrgPos() * radToRev,
                                    0.0, //radpsToRevpm * m.omegaMotor,
                                    m0.getRawPos(),
                                    m0.getEngrgVelEst() * radpsToRevpm,
                                    m0.getRawAct(),
                                    radToRev * pPID.GetSetpoint(),
                                    radToRev * pPID.GetError());
*/
            }
            else
            {
                MotorSim ms = mSim.get(i);
                // Simulation
            }

        }
        
    }

    public void InterfaceMotors(double tCur)
    {
        Motor m;
        int n = mtr.size();  // Number of motors

        for(int i = 0; i < n; i++)
        {
            m = mtr.get(i);
            // Connect motor data to simulation or real motor
            if(realMotors)
            {
                // Real
                m.setRawPos(-VisaIO.vioGetMotorPosition(m.posChan, 1.0),
                        tCur);
                VisaIO.vioSetMotorActuation(m.actChan, m.getRawAct());
            }
            else
            {
                MotorSim ms = mSim.get(i);
                // Simulation
                m.setEngrgPos(ms.angleMotor, tCur);
                m.setEngrgVelMeas(ms.omegaMotor);
                ms.v = m.getEngrgAct();
            }

        }
    }

    public void TurnOffMotors()
    {
        if(!realMotors)return;  // Nothing to
        Motor m;
        int n = mtr.size();  // Number of motors

        for(int i = 0; i < n; i++)
        {
            m = mtr.get(i);
            VisaIO.vioSetMotorActuation(m.actChan, 0.0);
        }
    }
}
