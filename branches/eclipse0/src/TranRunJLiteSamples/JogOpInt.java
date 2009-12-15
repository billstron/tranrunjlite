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

import TranRunJLite.*;
import java.util.ArrayList;
import lvcommpkg.*;

/** This Class/Task connects the robot jog operations to
 * a LabVIEW GUI
 * This task must not run any faster than the LabVIEW GUI loop -- otherwise
 * VERY jerky motion will result!
 * @author DMAuslander, Dec. 4, 2009
 */
public class JogOpInt extends TrjTask
{
    protected ArrayList<Jogger> joggerList;
    protected ArrayList<Motor> motorList;

    // Arrays used for communication with LV
    protected double [] jogV = new double[6];
    protected double [] pos = new double[6];
    protected double [] vel = new double[6];
    protected int [] switches = new int[6];
    protected boolean stop = false;

    public boolean getStopVal(){return stop;}
    
    // Variables that read from the LV library:
    protected int runMotorsFwdReader = 0;
    protected int stopReader = 0;
    protected int lv1ReadyReader = 0;
    protected int JogVelocityReader = 0;
    protected int RunMotorsRevReader = 0;
    protected int SetHomeReader = 0;

    // Variables that write to the LV library:
    protected int trj1ReadyWriter = 0;
    protected int trj1TimeWriter = 0;
    protected int MotorPositionsWriter = 0;
    protected int MotorVelocitiesWriter = 0;
    protected int FlagsWriter = 0;


    // States
    public static final int Init1 = 0;
    public static final int Init2 = 1;
    public static final int Jog = 2;
    public static final int Off = 3;

    // Commands
    public static final int Stop_Jogging = 0;

    protected double tNetDelay = 1.0;  // Delay to allow for network activity
    protected double t0 = 0.0;  // Used for timing inside states

    JogOpInt(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, 
            ArrayList<Jogger> joggerList, ArrayList<Motor> motorList)
    {
        super(name, sys, initialState, taskActive);
        this.dtNominal = dtNominal;
        this.joggerList = joggerList;
        this.motorList = motorList;
        stateNames.add("Init1");
        stateNames.add("Init2");
        stateNames.add("Jog");
        stateNames.add("Off");

        // Make sure all the jog tasks are activated and set initial
        // jog velocity to zero (to avoid unexpected motion)
        for(Jogger jj : joggerList)
        {
            jj.SetVelJog(0.0);
            jj.ActivateTask();
        }
    }

    public boolean RunTaskNow(TrjSys sys)
    {
        return CheckTime(sys.GetRunningTime());
    }


    private int [] runFwd = {0};
    private int [] runBack = {0};
    private int [] lvr = {0}; // Booleans come in from LV as int's

    public boolean RunTask(TrjSys sys)
    {
        int cmd;
        // Create link ("handle") variables to read from the LabVIEW library (note: these
        // are all int's regardless of the type of the underlying variable
        // because these variables are just the enabling link.


        double t = sys.GetRunningTime();
        nextState = -1;  // Default is stay in same state

        // **Usage note for LabVIEW network variables: all variables, whether arrays
        // or not, that are read by the Java program from LV
        // use memory references ("pointers" in C
        // terminology; the interface functions are written in C). To duplicate
        // that in Java all variables read from the network are
        // used as arrays; if they're scalar in
        // the Java program an array with a single element is used. This way, the
        // array name appears in the target C program as a memory reference.
        // For reading, regular values are used for scalars.

        switch(currentState)
        {
            case Init1: // Make the connections for network variables,
                  // then wait tNetDelay seconds before going to Init2 to complete
                  // initialization
            {
                if(runEntry)
                {
                    // Setup the network variables
                    LVComm.NViSetDebugPrint(1);  // Turn debugging printing on
                        // or off as needed
                    String hostIP = "\\\\localhost\\Robot2Lib\\";

                    for(int i = 0; i < 6; i++)jogV[i] = 0.0;  // Initialize

                    // Link variables to the LV library
                    runMotorsFwdReader = LVComm.NViCreateReader(hostIP, "runMotorsFwd");
                    stopReader = LVComm.NViCreateReader(hostIP, "stop");
                    lv1ReadyReader = LVComm.NViCreateReader(hostIP, "lv1Ready");
                    JogVelocityReader = LVComm.NViCreateReader(hostIP, "JogVelocity");
                    RunMotorsRevReader = LVComm.NViCreateReader(hostIP, "RunMotorsRev");
                    SetHomeReader = LVComm.NViCreateReader(hostIP, "SetHome");

                    trj1ReadyWriter = LVComm.NViCreateWriter(hostIP, "trj1Ready");
                    trj1TimeWriter = LVComm.NViCreateWriter(hostIP, "trj1Time");
                    MotorPositionsWriter = LVComm.NViCreateWriter(hostIP, "MotorPositions");
                    MotorVelocitiesWriter = LVComm.NViCreateWriter(hostIP, "MotorVelocities");
                    FlagsWriter = LVComm.NViCreateWriter(hostIP, "Flags");

                    t0 = t;  // Setup to time the wait
                }

                // No further actions here, just wait
                if((t - t0) >= tNetDelay)nextState = Init2;
                break;
            }  // End of Init1 state

            case Init2: // Complete the initialization  and wait for the
                // Ready signal from the LV GUI
            {
                if(runEntry)
                {
                    // Flush the buffers to assure valid initial values
                    LVComm.NViFlushBufferBool(runMotorsFwdReader, 5.0);
                    LVComm.NViFlushBufferBool(stopReader, 5.0);
                    LVComm.NViFlushBufferBool(lv1ReadyReader, 5.0);
                    LVComm.NViFlushBufferBool(RunMotorsRevReader, 5.0);
                    LVComm.NViFlushBufferBool(SetHomeReader, 5.0);
                }

                // Send READY signal to LabVIEW
                LVComm.NViSendBool(trj1ReadyWriter, 1);  // Booleans are
                    // 0 for false, not-0 for true
                 
                // Now wait for the LV Ready signal, then go on to Jog
                LVComm.NViGetBoolIfAvailable(lv1ReadyReader, lvr);
                if(lvr[0] != 0)nextState = Jog;
                break;
            }  // End of Init2 state

            case Jog:  // Read jogging commands from GUI and update GUI
            {
                LVComm.NViSetDebugPrint(0);  // Turn debugging printing on or off
                // Update time on the LV GUI
                LVComm.NViSendDouble(trj1TimeWriter, t);

                // Get the user input jog velocities from LV
                // Check for 'forward' or 'backward' signals
                runFwd[0] = 0;
                runBack[0] = 0;
                double sign = 1.0;
                LVComm.NViGetBoolIfAvailable(runMotorsFwdReader, runFwd);
                LVComm.NViGetBoolIfAvailable(RunMotorsRevReader, runBack);
                boolean fwd = (runFwd[0] != 0) ? true:false;
                boolean back = (runBack[0] != 0) ? true:false;
                //System.out.printf("runFwd %d fwd %b runBack %d back %b\n",
                //        runFwd[0], fwd, runBack[0], back);
                if(fwd || back)
                {
                    // Run the motors
                    if(back)sign= -1.0;
                    if(LVComm.NViGetDoubleArrayIfAvailable(JogVelocityReader, jogV) != 0)
                    {
                        for(int i = 0; i < 6; i++)
                        {
                            joggerList.get(i).SetVelJog(sign * jogV[i]);
                        }
                    }
                }
                else
                {
                    // For safety, set the jog velocity to zero if there is
                    // no new fwd/back information. However, doing this means
                    // that this task must runno faster than the LV GUI loop so
                    // that the motion is not jerky from all of the zeros.
                    for(int i = 0; i < 6; i++)
                    {
                        joggerList.get(i).SetVelJog(0.0);
                    }
                }

                // Send system information to the GUI
                for(int i = 0; i < 6; i++)
                {
                    pos[i] = motorList.get(i).getEngrgPos();
                    vel[i] = motorList.get(i).getEngrgVelEst();
                    switches[i] = (motorList.get(i).getSwitch(0)) ? 1 : 0;
                }
                LVComm.NViSendDoubleArray(MotorPositionsWriter, pos);
                LVComm.NViSendDoubleArray(MotorVelocitiesWriter, vel);
                LVComm.NViSendIntArray(FlagsWriter, switches);

                // Check for STOP signal from GUI
                int [] stopSig = {0};
                if(LVComm.NViGetBoolIfAvailable(stopReader, stopSig) != 0)
                {
                    if(stopSig[0] != 0)nextState = Off;
                    LVComm.NViSendBool(trj1ReadyWriter, 0);  // Clear READY signal
                            // so LV program will stop also.
                }
                break;
            }  // Endof Jog state

            case Off:
            {
                if(runEntry)
                {
                    t0 = t;  // Mark entry time
                }

                // Delay to allow for network activity, then set STOP
                if(t >= (t0 + tNetDelay))stop = true;
            }  // End of Off state
        }
        return false;  // Done for now
    }

}
