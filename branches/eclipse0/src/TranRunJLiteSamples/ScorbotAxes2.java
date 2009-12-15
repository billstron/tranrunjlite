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
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import TranRunJLite.*;
import visaio.*;

/** Collects information for a set of motors so that they can be
 * operated in a synchronized way (or, independently, if desired)
 * This version assumes that each motor has its own simulation
 * object. That is, that the motors are not dynamically connected. If
 * they are, a different structure would be needed.
 * This version is specific to the Scorbot robot and decouples the
 * wrist roll and pitch. It is based on MultiMotor.java
 * @author DMAuslander, Sept. 10, 09, modified for Scorbot, Oct. 1, 09
 */
public class ScorbotAxes2
{
    // ArrayLists are more convenient than plain arrays, but probably less
    // efficient.
    ArrayList<Motor> mtr = null;
    ArrayList<MotorSimScorbot2> mSim = null;
    ArrayList<SISOFeedback> fb = null;
    ArrayList<ProfileGenerator> prof = null;
    TrjSys sys = null;
    boolean realMotors;
    double [] encoderSign =null;
    double [] actuationSign =null;
    double [] motionSenseSign =null;

    public ScorbotAxes2(TrjSys sys, ArrayList<Motor> mtr,
            ArrayList<MotorSimScorbot2> mSim, ArrayList<SISOFeedback> fb,
            ArrayList<ProfileGenerator> prof, boolean realMotors,
            double [] encoderSign, double [] actuationSign, double [] motionSenseSign)
    {
        this.sys = sys;
        this.mtr = mtr;
        this.mSim = mSim;
        this.fb = fb;
        this.prof = prof;
        this.realMotors = realMotors;
        this.encoderSign = encoderSign;
        this.actuationSign = actuationSign;
        this.motionSenseSign = motionSenseSign;
    }

    public void AddMotor(Motor m)
    {
        mtr.add(m);
    }

    public void AddMotorSim(MotorSimScorbot2 ms)
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

        data[0] = t;
        int j = 1;
        for(int i = 0; i < n; i++)
        {
            m = mtr.get(i);
            // Connect motor data to simulation or real motor
            data[j++] = m.getEngrgPos();  // Motor position
            data[j++] = m.getEngrgVelMeas(); // Measured velocity
            data[j++] = m.getRawPos();
            data[j++] = m.getEngrgVelEst();
            data[j++] = m.getRawAct();
            if((fb.size() >= n) && (fb.get(i) != null))
            {
                data[j++] = fb.get(i).GetSetpoint();
                data[j++] = fb.get(i).GetError();
            }
            else
            {
                data[j++] = 0.0;
                data[j++] = 0.0;
            }
            data[j++] = (m.getSwitch(0))?1.0: 0.0;
        }        
    }

    public void WriteAllData(String fileName, double [][] data, int nData)
    {
        Motor m;
        PrintWriter f;

        int n = mtr.size();  // Number of motors
        int nCol = data[0].length;

        if(fileName != null)
        {
            // Write a file
            f = OpenDataFile(fileName);

            System.out.printf("<WriteAllData> data.length %d, data[0].length %d, n %d\n",
                    data.length, data[0].length, n);
            for(int i = 0; i < nData; i++)
            {
                for(int j = 0; j < nCol; j++)
                {
                    f.printf("%g ", data[i][j]);
                }
                f.printf("\n");
            }
            f.close();
        }
    }

    public PrintWriter OpenDataFile(String fileName)
    {
        // Set up a file for writing results
        PrintWriter dataFile0 = null;
        try
        {
            FileWriter fW = new FileWriter (fileName);
            dataFile0 = new PrintWriter ( fW );
        }
        catch(IOException e)
        {
            System.out.println("IO Error " + e);
            System.exit(1);  // File error -- quit
        }
        return dataFile0;
    }

    double encoder[] = new double[6];  // Use these for coupling the axis data
    double actuation[] = new double[6];
    double velocity[] = new double[6];  // Only used for simulation

    public void InterfaceMotors(double tCur)
    {
        Motor m;
        int n = mtr.size();  // Number of motors

        // The wrist pitch and roll axes are coupled on the robot
        // Decouple them here so all of the control acts directly
        // on pitch and roll.
        if(realMotors)
        {
            for(int i = 0; i < n; i++)
            {
                m = mtr.get(i);
                encoder[i] =
                        encoderSign[i] * VisaIO.vioGetMotorPosition(m.posChan, 1.0);
                boolean sw = (VisaIO.vioGetFlag(m.switchChan) != 0) ? true : false;
                m.setSwitch(0, sw);
                actuation[i] = actuationSign[i] * m.getRawAct();
            }

            // Check the coupled actuation axes for overflow due to
            // adding or subtracting
            double kf = 1.0;
            double enc = 0.0, act = 0.0;  // Local variables for encoder and actuation
            
            if((Math.abs(actuation[3]) + Math.abs(actuation[4])) > 1.0)
                kf = 1.0 / (Math.abs(actuation[3]) + Math.abs(actuation[4]));

            for(int i = 0; i < n; i++)
            {
                m = mtr.get(i);
                if(i == 3)
                {
                    enc = encoder[3] + encoder[4];
                    act = kf * (actuation[3] + actuation[4]);
                    //m.setRawPos(encoder[3] + encoder[4], tCur);
                    //VisaIO.vioSetMotorActuation(m.actChan, kf * (actuation[3] +
                    //        actuation[4]));
                }
                else if(i == 4)
                {
                    enc = encoder[3] - encoder[4];
                    act = kf * (actuation[3] - actuation[4]);
                    //m.setRawPos(encoder[3] - encoder[4], tCur);
                    //VisaIO.vioSetMotorActuation(m.actChan, kf * (actuation[3] -
                    //        actuation[4]));
                }
                else
                {
                    enc = encoder[i];
                    act = actuation[i];
                    //m.setRawPos(encoder[i], tCur);
                    //VisaIO.vioSetMotorActuation(m.actChan, actuation[i]);
                }
                m.setRawPos(motionSenseSign[i] * enc, tCur);
                VisaIO.vioSetMotorActuation(m.actChan, motionSenseSign[i] * act);
            }
        }
        else  // Simulation
        {
            for(int i = 0; i < n; i++)
            {
                m = mtr.get(i);
                MotorSimScorbot2 ms = mSim.get(i);
                encoder[i] = ms.pos;
                velocity[i] = ms.vel;
                actuation[i] = m.getEngrgAct();
            }

            // Check the coupled actuation axes for overflow due to
            // adding or subtracting
            double kf = 1.0;
            if((Math.abs(actuation[3]) + Math.abs(actuation[4])) > 1.0)
                kf = 1.0 / (Math.abs(actuation[3]) + Math.abs(actuation[4]));

            for(int i = 0; i < n; i++)
            {
                double pos = 0.0;
                m = mtr.get(i);
                MotorSimScorbot2 ms = mSim.get(i);
                if(i == 3)
                {
                    pos = encoder[3] + encoder[4];
                    m.setEngrgPos(pos, tCur);
                    m.setEngrgVelMeas(velocity[3] + velocity[4]);
                    ms.u =  kf * (actuation[3] + actuation[4]);
                }
                else if(i == 4)
                {
                    pos = encoder[3] - encoder[4];
                    m.setEngrgPos(pos, tCur);
                    m.setEngrgVelMeas(velocity[3] - velocity[4]);
                    ms.u =  kf * (actuation[3] - actuation[4]);
                }
                else
                {
                    pos = encoder[i];
                    m.setEngrgPos(pos, tCur);
                    m.setEngrgVelMeas(velocity[i]);
                    ms.u =  actuation[i];
                }
                // The Scorbot homing switches are connected to the wrist
                // rotation and pitch motions rather than to the driving
                // motors. Thus, switch simulation is here rather than in the
                // simulation module.
                if((pos >= ms.sw0) && (pos <= ms.sw1))m.setSwitch(0, true);
                else m.setSwitch(0, false);
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
