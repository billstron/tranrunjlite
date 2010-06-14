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
import TranRunJLite.PerfTimer.*;
import TranRunJLiteSamples.ScorbotSamples.MotorCosineProfile;
import visaio.*;
//import java.io.FileWriter;
import java.io.IOException;
//import java.io.PrintWriter;
import java.util.ArrayList;

/** TranRunJLite Scorbot Samples
 * @author DMAuslander, Sept 29, 2009
 */
public class ScorbotJogHome
{
    enum Samples{JogTest, JogWithGUI, HomingTest, HomingWithGUI};
    
    static final double degPerRad = 180.0 /Math.PI;
    static final double radPerDeg = 1.0 / degPerRad;
    static final double radPerRev = 2.0 * Math.PI;
    static final double revPerRad = 1.0 / radPerRev;
    static final double radPsPerRevPM = radPerRev / 60.0;
    static final double revPmPerRadPs = 1.0 / radPsPerRevPM;

    // Conversion factors for encoders (order is opposite numbering here!)
    // Encoder resolution, counts/deg (except for gripper, motor 0, counts/cm)
    //static double encodeRes[] = {601.0, 27.8, 27.8, 110.0, 120.0, 141.14};

    boolean realMotors = false;  // Set operating mode
    boolean realTime = false;
    boolean doLogging = true;  // Whether or not to create a
       //logging task
    
    MotorSimScorbot2 m = null;
    PosPID pPID = null;
    MotorCosineProfileP mProfCosine = null;
    DataLoggerScorbot2 log = null;
    ScorbotJogger jog = null;
    JogToEvent jogEvent = null;
    JogOpInt jogGUI = null;
    HomeOpInt homingGUI = null;
    NewJog newJog = null;
    Homing homing = null;

    // To collect information (ArrayLists are more convenient to use than
    // arrays, although they are probably less efficient).
    ArrayList<Motor> mtr = new ArrayList<Motor>();
    ArrayList<MotorSimScorbot2> mSim = new ArrayList<MotorSimScorbot2>();
    ArrayList<SISOFeedback> fb = new ArrayList<SISOFeedback>();
    ArrayList<MotorCosineProfileP> mcpp = new ArrayList<MotorCosineProfileP>();
    ArrayList<ProfileGenerator> prof = new ArrayList<ProfileGenerator>();
    ArrayList<Jogger> jogList = new ArrayList<Jogger>();
    ArrayList<JogToEvent> jogEventList = new ArrayList<JogToEvent>();
    ArrayList<NewJog> newJogList  = new ArrayList<NewJog>();

    /** This package has a set of TranRunJLite sample programs
     * Each section of the switch() is a separate sample program
     * Use selection variable to decide which sample to run
     * @param args the command line arguments (not used)
     */
    public static void main(String[] args)
    {
        ScorbotJogHome main = new ScorbotJogHome();
        main.RunSamples();
    }

    public void RunSamples()
    {
        // Select the problem to be solved
        Samples s = Samples.JogTest;
        //Samples s = Samples.JogWithGUI;
        //Samples s = Samples.HomingTest;
        //Samples s = Samples.HomingWithGUI;
        
        TrjSys sys;
        double dt = 0.0;  // Used for samples that need a time delta
        double tFinal = 0.0;
        double dtLog = 0.0, dtVel = 0.0;

        // Set up a file for writing results
        //PrintWriter dataFile0 = null;
        
        switch(s)
        {
            case JogTest:
            {
                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   deg            counts       deg  (of link motion)
                //   cm             counts       cm  (for gripper)
                //   deg/s          counts/s     deg/s (cm/s for gripper)
                //     Actuation
                //   duty cycle          duty cycle   duty cycle

                boolean realMode = true;
                realMotors = realMode;  // Set for operating mode
                realTime = realMode;

                TrjTime timeKeeper = null;

                // Create a time object
                if(realTime)
                {
                    timeKeeper = new TrjTimeReal();
                }
                else
                {
                    timeKeeper = new TrjTimeSim(0.0);
                }

                sys = new TrjSys(timeKeeper);

                // These are +1 or -1 values to map actual wiring and
                // motion directions to desired
                // The encoder and actuation signs are physical and affect
                // feedback loop stability.
                // The motion sense provides for mapping to get better
                // user intuition about how a system operates.
                // For Berkeley modified ER-V (Gray):
                double [] encoderSign = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
                // For Berkeley modified ER-4u:
                //double [] encoderSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

                double [] actuationSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
                double [] motionSenseSign = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

                ScorbotAxes2 multi = new ScorbotAxes2(
                        sys, // TrjSys sys,
                        mtr, // ArrayList<MotorSimScorbot2> mSim,
                        mSim, // ArrayList<Motor> mtr,
                        fb, // ArrayList<SISOFeedback> fb,
                        prof, // ArrayList<ProfileGenerator> prof
                        realMotors, // boolean realMotors,
                        encoderSign, // double [] encoderSign,
                        actuationSign, //double [] actuationSign,
                        motionSenseSign //double [] motionSenseSign
                        );

                // Axis scaling (counts/deg except for gripper, counts/cm)
                double [] scale = {141.14, 120.0, 110.0, 55.0, 55.0, 601.0};
                // wrist: ch 3 is roll, ch 4 is pitch
                double [] motorActuation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                //double [] motorActuation = {-0.0, 0.0, 0.4, 0.0, 0.0, 0.0};
                dtVel = 0.005;
                int [] motorChan = {5, 4, 3, 2, 1, 0};  // Which channel to use for this motor
                int [] switchChan = {5, 4, 3, 1, 2, 0};  // Which channel to use for this motor
                Motor m0 = null;
                for(int i = 0; i < 6; i++)
                {
                    m0 = new Motor(
                            0.0, // double r0 (initial raw position)
                            scale[i], // double engrgToRawPos, counts/deg
                            scale[i], // or counts/cm for gripper
                            0.0, // double actRaw0
                            1.0, // double engrgToRawAct (duty cycle for both)
                            dtVel, // double dtVel
                            motorChan[i], // int posChan
                            motorChan[i], // int actChan
                            switchChan[i], // int switchChan
                            1 // int nSwitches
                            );
                    mtr.add(m0);
                    m0.setEngrgAct(motorActuation[i]);
                }

                if(!realMotors)
                {
                    // Only instantiate for simulated operation
                    String [] axisNames = {"Waist", "Upper Arm", "Fore Arm",
                       "Wrist Roll", "Wrist Pitch", "Gripper"};
                    double [] k1Vals = {1.0 / 0.03, 1.0 / 0.015, 1.0 / 0.025,
                        1.0 / 0.026, 1.0 / 0.026, 1.0 / 0.1};
                    // The shoulder and elbow axes are affected by gravity so
                    // some values here have been changed to match particular
                    // experiments with the robot.
                    //double [] gainVals = {25.0, 13.3, 28.9, 61.9, 61.9, 6.2};
                    double [] gainVals = {25.0, 25.0, 28.9, 61.9, 61.9, 6.2};
                    double [] sw0Vals = {-12.0, -40.0, 50.0, 60.0, -20.0, -1.0e10};
                    double [] sw1Vals = {-10.0, -30.0, 70.0, 65.5, 1.0e10, 1.0e10};
                    double [] hsNegVals = {-1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10};
                    double [] hsPosVals = {1.0e10, 1.0e10, 1.0e10, 1.0e10, 1.0e10, 4.0};

                    for(int i = 0; i < 6; i++)
                    {
                        m = new MotorSimScorbot2(
                                axisNames[i], //String name,
                                sys, //TrjSys sys,
                                0.0, //double u,  // PWM duty cycle
                                k1Vals[i], //double k1, // 1/time constant, 1/sec
                                gainVals[i], //double ssGain, // steady state gain
                                  // for gripper, (cm/s)/(duty cycle)
                                sw0Vals[i], //double sw0, // Bounding locations for switch
                                sw1Vals[i], //double sw1,
                                false, //boolean invertSwitch,
                                hsNegVals[i], //double hsNeg, // Hardstop locations
                                hsPosVals[i], //double hsPos,
                                1.0e2, //double khs,  // Spring constant for hardstop
                                true // boolean useAdaptiverSolver
                                );
                        mSim.add(m);
                    }
                }

                // Create PID controllers
                // See below for the definition of this "inner" class
                for(int i = 0; i < 6; i++)
                {
                    String [] axisNames = {"WaistPID", "Upper ArmPID",
                        "Fore ArmPID",
                        "Wrist RollPID", "Wrist PitchPID", "GripperPID"};
                    // Note -- the two sets of kp, kd are for simulation
                    // tuned (first) and real robot tuned (second)
                    double [] kpVals = {3.0, 4.0, 4.0, 1.0, 1.0, 10.0};
                    //double [] kpVals = {8.0, 12.0, 12.0, 3.5, 3.5, 150.0};
                    double [] kiVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    double [] kdVals = {0.0, 0.0, 0.02, 0.0, 0.0, 0.0};
                    //double [] kdVals = {0.1, 0.1, 0.1, 0.03, 0.03, 3.0};
                    double [] setVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    pPID = new PosPID(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            0.005, //double dt, sec
                            -1.0, //double mMin, volts
                            1.0, //double mMax,
                            0.0, //double mOff,
                            1, //int initialState, start with control on
                            true, //boolean taskActive,
                            kpVals[i], //double kp,
                            kiVals[i], //double ki,
                            kdVals[i], //double kd,
                            0.0, //double integ0,
                            true, //boolean triggerMode,
                            true, //boolean useAntiWindup
                            mtr.get(i) // Motor mot
                            );

                    pPID.SetStateTracking(true);
                    pPID.SetSetpoint(setVals[i]);  // deg (cm for gripper)
                    fb.add(pPID);
                }

                // Put this here so it's in scope during control
                //double [] velJog = {-20.0, -20.0, 20.0, 30.0, -30.0, 2.0};
                double [] velJog = {5.0, 10.0, 4.0, 2.5, -10.0, -1.0};
                //double [] velJog = {0.0, 0.0, 0.0, 0.0, 20.0, 0.0};
                
                for(int i = 0; i < 6; i++)
                {
                    // See below for the definition of this "inner" class
                    String [] axisNames = {"WaistProf", "Upper ArmProf",
                        "Fore ArmProf",
                        "Wrist RollProf", "Wrist PitchProf", "GripperProf"};
                    double [] sNeg = {-180.0, -180.0,-180.0,-180.0,-180.0, -10.0};
                    double [] sPos = {180.0, 180.0, 180.0, 180.0, 180.0, 10.0};
                    
                    jog =new ScorbotJogger(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            true, //boolean taskActive,
                            0.005, //0.001, //double dtNominal,
                            0.0, //double s0,
                            sNeg[i], //double sNeg,
                            sPos[i], //double sPos,
                            velJog[i], //double velJog
                            fb.get(i) // SISOFeedback cntlr
                            );

                    jogList.add(jog);
                }

                dt = 1.0e-5;  // For simulated time

                tFinal = 6.0; //8.0; // Use the longer profile for an up and back move
                dtLog = 2.0e-3;
                // Create a data logging task
                log = new DataLoggerScorbot2
                        (
                        "DataLog1", //String name,
                        sys, //TrjSys sys,
                        0, //int initialState, (no states in this task)
                        true, //boolean taskActive,
                        dtLog, //double dt,
                        multi, //MultiMotor mm,
                        0.0, //double ts,
                        tFinal //double te
                        );
                double tNextJogVel = 3.0;
                boolean nextJogVelSet = false;

                if(realMotors)
                {
                    VisaIO.vioInit();
                }

                // Create a timing histogram
                Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);
                double tPrev = 0.0;
                boolean first = true;  // True during first run through loop

                timeKeeper.resetRunningTime();
                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    double dtLoop = tCur - tPrev;
                    if(dtLoop > 0.008)
                        System.out.println("t dtLoop " + tCur + " " + dtLoop);
                    if(!first)h.HistValue(dtLoop);  // Ignore the first pass

                    if((tCur > tNextJogVel) && !nextJogVelSet)
                    {
                        nextJogVelSet = true;
                        for(int i = 0; i < 6; i++)
                        {
                            // Bring it back to the original position
                            jogList.get(i).SetVelJog(-velJog[i]);
                        }
                    }

                    // Connect motor data to simulation or real motor
                    multi.InterfaceMotors(tCur);

                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    // Data logging has been replaced by a separate task
                    sys.IncrementRunningTime(dt);
                    first = false;
                    tPrev = tCur;
                }
                multi.TurnOffMotors();
                h.WriteHistogram("HistPtToPt.txt");
                break;
            }

            case JogWithGUI:
            {
                // The difference between this example and the JogTest is that
                // in this case the jog velocity is continuously controlled
                // by the user with a LabVIEW GUI
                
                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   deg            counts       deg  (of link motion)
                //   cm             counts       cm  (for gripper)
                //   deg/s          counts/s     deg/s (cm/s for gripper)
                //     Actuation
                //   duty cycle          duty cycle   duty cycle

                boolean realMode = true;
                realMotors = realMode;  // Set for operating mode
                realTime = realMode;

                TrjTime timeKeeper = null;

                // Create a time object
                if(realTime)
                {
                    timeKeeper = new TrjTimeReal();
                }
                else
                {
                    timeKeeper = new TrjTimeSim(0.0);
                }

                sys = new TrjSys(timeKeeper);

                // These are +1 or -1 values to map actual wiring and
                // motion directions to desired
                // The encoder and actuation signs are physical and affect
                // feedback loop stability.
                // The motion sense provides for mapping to get better
                // user inituition about how a system operates.
                
                // Robot Axes:
                // 0 Waist; 1 Shoulder; 2 Elbow; 3 Wrist Rotation;
                // 4 Wrist Pitch; 5 Gripper
                // For Berkeley modified ER-V (Gray):
                double [] encoderSign = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
                // For Berkeley modified ER-4u:
                //double [] encoderSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

                double [] actuationSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
                // Motion sense controls the sign convention for the
                // direction of actual motion (ie, what direction of robot
                // motion corresponds to positive position). The
                // conventions for positive motion are:
                // Waist: clockwise (from above)
                // Shoulder, elbow, wrist pitch: down
                // Wrist rotation: clockwise (looking into the end effector)
                // Gripper: close
                double [] motionSenseSign = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

                ScorbotAxes2 multi = new ScorbotAxes2(
                        sys, // TrjSys sys,
                        mtr, // ArrayList<MotorSimScorbot2> mSim,
                        mSim, // ArrayList<Motor> mtr,
                        fb, // ArrayList<SISOFeedback> fb,
                        prof, // ArrayList<ProfileGenerator> prof
                        realMotors, // boolean realMotors,
                        encoderSign, // double [] encoderSign,
                        actuationSign, //double [] actuationSign,
                        motionSenseSign //double [] motionSenseSign
                        );

                // Axis scaling (counts/deg except for gripper, counts/cm)
                double [] scale = {141.14, 120.0, 110.0, 55.0, 55.0, 601.0};
                // wrist: ch 3 is roll, ch 4 is pitch
                double [] motorActuation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                //double [] motorActuation = {-0.0, 0.0, 0.4, 0.0, 0.0, 0.0};
                dtVel = 0.005;
                int [] motorChan = {5, 4, 3, 2, 1, 0};  // Which channel to use for this motor
                int [] switchChan = {5, 4, 3, 1, 2, 0};  // Which channel to use for this motor
                Motor m0 = null;
                for(int i = 0; i < 6; i++)
                {
                    m0 = new Motor(
                            0.0, // double r0 (initial raw position)
                            scale[i], // double engrgToRawPos, counts/deg
                            scale[i], // or counts/cm for gripper
                            0.0, // double actRaw0
                            1.0, // double engrgToRawAct (duty cycle for both)
                            dtVel, // double dtVel
                            motorChan[i], // int posChan
                            motorChan[i], // int actChan
                            switchChan[i], // int switchChan
                            1 // int nSwitches
                            );
                    mtr.add(m0);
                    m0.setEngrgAct(motorActuation[i]);
                }

                if(!realMotors)
                {
                    // Only instantiate for simulated operation
                    String [] axisNames = {"Waist", "Upper Arm", "Fore Arm",
                       "Wrist Roll", "Wrist Pitch", "Gripper"};
                    double [] k1Vals = {1.0 / 0.03, 1.0 / 0.015, 1.0 / 0.025,
                        1.0 / 0.026, 1.0 / 0.026, 1.0 / 0.1};
                    // The shoulder and elbow axes are affected by gravity so
                    // some values here have been changed to match particular
                    // experiments with the robot.
                    //double [] gainVals = {25.0, 13.3, 28.9, 61.9, 61.9, 6.2};
                    double [] gainVals = {25.0, 25.0, 28.9, 61.9, 61.9, 6.2};
                    double [] sw0Vals = {-12.0, -40.0, 50.0, 60.0, -20.0, -1.0e10};
                    double [] sw1Vals = {-10.0, -30.0, 70.0, 65.5, 1.0e10, 1.0e10};
                    double [] hsNegVals = {-1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10};
                    double [] hsPosVals = {1.0e10, 1.0e10, 1.0e10, 1.0e10, 1.0e10, 4.0};

                    for(int i = 0; i < 6; i++)
                    {
                        m = new MotorSimScorbot2(
                                axisNames[i], //String name,
                                sys, //TrjSys sys,
                                0.0, //double u,  // PWM duty cycle
                                k1Vals[i], //double k1, // 1/time constant, 1/sec
                                gainVals[i], //double ssGain, // steady state gain
                                  // for gripper, (cm/s)/(duty cycle)
                                sw0Vals[i], //double sw0, // Bounding locations for switch
                                sw1Vals[i], //double sw1,
                                false, //boolean invertSwitch,
                                hsNegVals[i], //double hsNeg, // Hardstop locations
                                hsPosVals[i], //double hsPos,
                                1.0e2, //double khs,  // Spring constant for hardstop
                                true // boolean useAdaptiverSolver
                                );
                        mSim.add(m);
                    }
                }

                // Create PID controllers
                // See below for the definition of this "inner" class
                for(int i = 0; i < 6; i++)
                {
                    String [] axisNames = {"WaistPID", "Upper ArmPID",
                        "Fore ArmPID",
                        "Wrist RollPID", "Wrist PitchPID", "GripperPID"};
                    // Note -- the two sets of kp, kd are for simulation
                    // tuned (first) and real robot tuned (second)
                    double [] kpVals = {3.0, 4.0, 4.0, 1.0, 1.0, 10.0};
                    //double [] kpVals = {8.0, 12.0, 12.0, 3.5, 3.5, 150.0};
                    double [] kiVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    double [] kdVals = {0.0, 0.0, 0.02, 0.0, 0.0, 0.0};
                    //double [] kdVals = {0.1, 0.1, 0.1, 0.03, 0.03, 3.0};
                    double [] setVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    pPID = new PosPID(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            0.005, //double dt, sec
                            -1.0, //double mMin, volts
                            1.0, //double mMax,
                            0.0, //double mOff,
                            1, //int initialState, start with control on
                            true, //boolean taskActive,
                            kpVals[i], //double kp,
                            kiVals[i], //double ki,
                            kdVals[i], //double kd,
                            0.0, //double integ0,
                            true, //boolean triggerMode,
                            true, //boolean useAntiWindup
                            mtr.get(i) // Motor mot
                            );

                    pPID.SetStateTracking(true);
                    pPID.SetSetpoint(setVals[i]);  // deg (cm for gripper)
                    fb.add(pPID);
                }

                // Put this here so it's in scope during control
                // Make sure there is no movement until the user commands
                // movement from the GUI
                double [] velJog = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                for(int i = 0; i < 6; i++)
                {
                    // See below for the definition of this "inner" class
                    String [] axisNames = {"WaistProf", "Upper ArmProf",
                        "Fore ArmProf",
                        "Wrist RollProf", "Wrist PitchProf", "GripperProf"};
                    double [] sNeg = {-180.0, -180.0,-180.0,-180.0,-180.0, -10.0};
                    double [] sPos = {180.0, 180.0, 180.0, 180.0, 180.0, 10.0};

                    jog =new ScorbotJogger(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            true, //boolean taskActive,
                            0.005, //0.001, //double dtNominal,
                            0.0, //double s0,
                            sNeg[i], //double sNeg,
                            sPos[i], //double sPos,
                            velJog[i], //double velJog
                            fb.get(i) // SISOFeedback cntlr
                            );

                    jogList.add(jog);
                }

                dt = 1.0e-5;  // For simulated time

                tFinal = 1.0e10; //Run unti user sends STOP signal from GUI
                dtLog = 2.0e-3;
                doLogging = false;
                if(doLogging)
                {
                    // Create a data logging task
                    log = new DataLoggerScorbot2
                            (
                            "DataLog1", //String name,
                            sys, //TrjSys sys,
                            0, //int initialState, (no states in this task)
                            true, //boolean taskActive,
                            dtLog, //double dt,
                            multi, //MultiMotor mm,
                            0.0, //double ts,
                            tFinal //double te
                            );
                }

                // Make a task for the GUI
                jogGUI = new JogOpInt(
                        "JogGUI", //String name, 
                        sys, //TrjSys sys, 
                        JogOpInt.Init1, //int initialState,
                        true, //boolean taskActive, 
                        0.050, //double dtNominal (sec),
                        jogList, //ArrayList<Jogger> joggerList,
                        mtr //ArrayList<Motor> motorList
                        );
                
                if(realMotors)
                {
                    VisaIO.vioInit();
                }

                // Create a timing histogram
                Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);
                double tPrev = 0.0;
                boolean first = true;  // True during first run through loop

                timeKeeper.resetRunningTime();
                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    double dtLoop = tCur - tPrev;
                    if(dtLoop > 0.008)
                        System.out.println("t dtLoop " + tCur + " " + dtLoop);
                    if(!first)h.HistValue(dtLoop);  // Ignore the first pass

                    // Connect motor data to simulation or real motor
                    multi.InterfaceMotors(tCur);

                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    // Data logging has been replaced by a separate task
                    sys.IncrementRunningTime(dt);
                    first = false;
                    tPrev = tCur;

                    // Check to see if user has signalled to stop
                    if(jogGUI.getStopVal())break;
                }
                multi.TurnOffMotors();
                h.WriteHistogram("HistPtToPt.txt");
                break;
            }

            case HomingTest:
            {
                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   deg            counts       deg  (of link motion)
                //   cm             counts       cm  (for gripper)
                //   deg/s          counts/s     deg/s (cm/s for gripper)
                //     Actuation
                //   duty cycle          duty cycle   duty cycle

                boolean realMode = false;
                realMotors = realMode;  // Set for operating mode
                realTime = realMode;

                TrjTime timeKeeper = null;

                // Create a time object
                if(realTime)
                {
                    timeKeeper = new TrjTimeReal();
                }
                else
                {
                    timeKeeper = new TrjTimeSim(0.0);
                }

                sys = new TrjSys(timeKeeper);

                // These are +1 or -1 values to map actual wiring and
                // motion directions to desired
                // The encoder and actuation signs are physical and affect
                // feedback loop stability.
                // The motion sense provides for mapping to get better
                // user intuition about how a system operates.
                // For Berkeley modified ER-V (Gray):
                double [] encoderSign = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
                // For Berkeley modified ER-4u:
                //double [] encoderSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

                double [] actuationSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
                double [] motionSenseSign = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

                ScorbotAxes2 multi = new ScorbotAxes2(
                        sys, // TrjSys sys,
                        mtr, // ArrayList<MotorSimScorbot2> mSim,
                        mSim, // ArrayList<Motor> mtr,
                        fb, // ArrayList<SISOFeedback> fb,
                        prof, // ArrayList<ProfileGenerator> prof
                        realMotors, // boolean realMotors,
                        encoderSign, // double [] encoderSign,
                        actuationSign, //double [] actuationSign,
                        motionSenseSign //double [] motionSenseSign
                        );

                // Axis scaling (counts/deg except for gripper, counts/cm)
                double [] scale = {141.14, 120.0, 110.0, 55.0, 55.0, 601.0};
                // wrist: ch 3 is roll, ch 4 is pitch
                double [] motorActuation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                //double [] motorActuation = {-0.0, 0.0, 0.4, 0.0, 0.0, 0.0};
                dtVel = 0.005;
                int [] motorChan = {5, 4, 3, 2, 1, 0};  // Which channel to use for this motor
                int [] switchChan = {5, 4, 3, 1, 2, 0};  // Which channel to use for this motor
                Motor m0 = null;
                for(int i = 0; i < 6; i++)
                {
                    m0 = new Motor(
                            0.0, // double r0 (initial raw position)
                            scale[i], // double engrgToRawPos, counts/deg
                            scale[i], // or counts/cm for gripper
                            0.0, // double actRaw0
                            1.0, // double engrgToRawAct (duty cycle for both)
                            dtVel, // double dtVel
                            motorChan[i], // int posChan
                            motorChan[i], // int actChan
                            switchChan[i], // int switchChan
                            1 // int nSwitches
                            );
                    mtr.add(m0);
                    m0.setEngrgAct(motorActuation[i]);
                }

                if(!realMotors)
                {
                    // Only instantiate for simulated operation
                    String [] axisNames = {"Waist", "Upper Arm", "Fore Arm",
                       "Wrist Roll", "Wrist Pitch", "Gripper"};
                    double [] k1Vals = {1.0 / 0.03, 1.0 / 0.015, 1.0 / 0.025,
                        1.0 / 0.026, 1.0 / 0.026, 1.0 / 0.1};
                    // The shoulder and elbow axes are affected by gravity so
                    // some values here have been changed to match particular
                    // experiments with the robot.
                    //double [] gainVals = {25.0, 13.3, 28.9, 61.9, 61.9, 6.2};
                    double [] gainVals = {25.0, 25.0, 28.9, 61.9, 61.9, 6.2};
                    double [] sw0Vals = {-12.0, -40.0, 50.0, 60.0, -20.0, -1.0e10};
                    double [] sw1Vals = {-10.0, -30.0, 70.0, 65.5, 1.0e10, 1.0e10};
                    double [] hsNegVals = {-1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10, -4.0};
                    double [] hsPosVals = {1.0e10, 1.0e10, 1.0e10, 1.0e10, 1.0e10, 4.0};

                    for(int i = 0; i < 6; i++)
                    {
                        m = new MotorSimScorbot2(
                                axisNames[i], //String name,
                                sys, //TrjSys sys,
                                0.0, //double u,  // PWM duty cycle
                                k1Vals[i], //double k1, // 1/time constant, 1/sec
                                gainVals[i], //double ssGain, // steady state gain
                                  // for gripper, (cm/s)/(duty cycle)
                                sw0Vals[i], //double sw0, // Bounding locations for switch
                                sw1Vals[i], //double sw1,
                                false, //boolean invertSwitch,
                                hsNegVals[i], //double hsNeg, // Hardstop locations
                                hsPosVals[i], //double hsPos,
                                1.0e2, //double khs,  // Spring constant for hardstop
                                true // boolean useAdaptiverSolver
                                );
                        mSim.add(m);
                    }
                }

                // Create PID controllers
                // See below for the definition of this "inner" class
                for(int i = 0; i < 6; i++)
                {
                    String [] axisNames = {"WaistPID", "Upper ArmPID",
                        "Fore ArmPID",
                        "Wrist RollPID", "Wrist PitchPID", "GripperPID"};
                    // Note -- the two sets of kp, kd are for simulation
                    // tuned (first) and real robot tuned (second)
                    double [] kpVals = {3.0, 4.0, 4.0, 1.0, 1.0, 10.0};
                    //double [] kpVals = {8.0, 12.0, 12.0, 3.5, 3.5, 150.0};
                    double [] kiVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    double [] kdVals = {0.0, 0.0, 0.02, 0.0, 0.0, 0.0};
                    //double [] kdVals = {0.1, 0.1, 0.1, 0.03, 0.03, 3.0};
                    double [] setVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    pPID = new PosPID(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            0.005, //double dt, sec
                            -1.0, //double mMin, volts
                            1.0, //double mMax,
                            0.0, //double mOff,
                            1, //int initialState, start with control on
                            true, //boolean taskActive,
                            kpVals[i], //double kp,
                            kiVals[i], //double ki,
                            kdVals[i], //double kd,
                            0.0, //double integ0,
                            true, //boolean triggerMode,
                            true, //boolean useAntiWindup
                            mtr.get(i) // Motor mot
                            );

                    pPID.SetStateTracking(true);
                    pPID.SetSetpoint(setVals[i]);  // deg (cm for gripper)
                    fb.add(pPID);
                }

                // Put this here so it's in scope during control
                //double [] velJog = {0.0, 2.0, 2.0, 0.0, 0.0, 0.0};
                double [] velJog = {-5.0, -10.0, 14.0, 12.5, -10.0, -1.0};
                //double [] velJog = {0.0, 0.0, 0.0, 0.0, 20.0, 0.0};
                
                for(int i = 0; i < 6; i++)
                {
                    // See below for the definition of this "inner" class
                    String [] axisNames = {"WaistProf", "Upper ArmProf",
                        "Fore ArmProf",
                        "Wrist RollProf", "Wrist PitchProf", "GripperProf"};
                    double [] sNeg = {-180.0, -180.0,-180.0,-180.0,-180.0, -10.0};
                    double [] sPos = {180.0, 180.0, 180.0, 180.0, 180.0, 10.0};
                    boolean [] lookFor = {true, true, true, true, false, false};
                    EventType [] eType = {EventType.Switch, EventType.Switch, EventType.Switch, 
                    		EventType.Switch, EventType.Switch, EventType.Hardstop};
                    
                    jogEvent =new JogToEvent(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            true, //boolean taskActive,
                            0.005, //0.001, //double dtNominal,
                            0.0, //double s0,
                            sNeg[i], //double sNeg,
                            sPos[i], //double sPos,
                            velJog[i], //double velJog
                            fb.get(i), // SISOFeedback cntlr
                            mtr.get(i),  //Motor mtr,
                            true, //boolean stopOnEvent, 
                            lookFor[i], //boolean switchLookForValue
                            0.2,  //double errorVal, 
                            eType[i] //EventType eType
                            );

                    jogEventList.add(jogEvent);
                }

                dt = 1.0e-5;  // For simulated time

                tFinal = 5.0; 
                dtLog = 2.0e-3;
                // Create a data logging task
                log = new DataLoggerScorbot2
                        (
                        "DataLog1", //String name,
                        sys, //TrjSys sys,
                        0, //int initialState, (no states in this task)
                        true, //boolean taskActive,
                        dtLog, //double dt,
                        multi, //MultiMotor mm,
                        0.0, //double ts,
                        tFinal //double te
                        );

                if(realMotors)
                {
                    VisaIO.vioInit();
                }

                // Create a timing histogram
                Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);
                double tPrev = 0.0;
                boolean first = true;  // True during first run through loop

                timeKeeper.resetRunningTime();
                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    double dtLoop = tCur - tPrev;
                    if(dtLoop > 0.008)
                        System.out.println("t dtLoop " + tCur + " " + dtLoop);
                    if(!first)h.HistValue(dtLoop);  // Ignore the first pass

                    // Connect motor data to simulation or real motor
                    multi.InterfaceMotors(tCur);

                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    sys.IncrementRunningTime(dt);
                    first = false;
                    tPrev = tCur;
                }
                multi.TurnOffMotors();
                // Print homing results
                System.out.printf("Axis Homed Time Location\n");
                for(int i = 0; i < 6; i++)
                {
                	System.out.printf("%d %b %g %g\n", i, jogEventList.get(i).GetEventFlag(),
                			jogEventList.get(i).GetEventTime(),
                			jogEventList.get(i).GetEventLocation());
                }               
                h.WriteHistogram("HistPtToPt.txt");
                break;
            }     
            
            case HomingWithGUI:
            {
            	// Full homing -- 
            	// 1) Allow user to jog robot to proper initial position
            	// 2) Use a profile to move to the pre-home position
            	// 3) Perform the homing (taking care of axis order)
            	// 4) Use a profile move to go to the "park" position
            	// All of this is done in the task HomeOpInt so this section is
            	// mostly setup.
            	
            	// To do this the task Jogger is replaced by Jogger2 with additional
            	// states to allow for turning it on and off.
                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   deg            counts       deg  (of link motion)
                //   cm             counts       cm  (for gripper)
                //   deg/s          counts/s     deg/s (cm/s for gripper)
                //     Actuation
                //   duty cycle          duty cycle   duty cycle

                boolean realMode = true;
                realMotors = realMode;  // Set for operating mode
                realTime = realMode;

                TrjTime timeKeeper = null;

                // Create a time object
                if(realTime)
                {
                    timeKeeper = new TrjTimeReal();
                }
                else
                {
                    timeKeeper = new TrjTimeSim(0.0);
                }

                sys = new TrjSys(timeKeeper);

                // These are +1 or -1 values to map actual wiring and
                // motion directions to desired
                // The encoder and actuation signs are physical and affect
                // feedback loop stability.
                // The motion sense provides for mapping to get better
                // user inituition about how a system operates.
                
                // Robot Axes:
                // 0 Waist; 1 Shoulder; 2 Elbow; 3 Wrist Rotation;
                // 4 Wrist Pitch; 5 Gripper
                // For Berkeley modified ER-V (Gray):
                double [] encoderSign = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
                // For Berkeley modified ER-4u:
                //double [] encoderSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

                double [] actuationSign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
                // Motion sense controls the sign convention for the
                // direction of actual motion (ie, what direction of robot
                // motion corresponds to positive position). The
                // conventions for positive motion are:
                // Waist: clockwise (from above)
                // Shoulder, elbow, wrist pitch: down
                // Wrist rotation: clockwise (looking into the end effector)
                // Gripper: close
                double [] motionSenseSign = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

                ScorbotAxes2 multi = new ScorbotAxes2(
                        sys, // TrjSys sys,
                        mtr, // ArrayList<MotorSimScorbot2> mSim,
                        mSim, // ArrayList<Motor> mtr,
                        fb, // ArrayList<SISOFeedback> fb,
                        prof, // ArrayList<ProfileGenerator> prof
                        realMotors, // boolean realMotors,
                        encoderSign, // double [] encoderSign,
                        actuationSign, //double [] actuationSign,
                        motionSenseSign //double [] motionSenseSign
                        );

                // Axis scaling (counts/deg except for gripper, counts/cm)
                double [] scale = {141.14, 120.0, 110.0, 55.0, 55.0, 601.0};
                // wrist: ch 3 is roll, ch 4 is pitch
                double [] motorActuation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                //double [] motorActuation = {-0.0, 0.0, 0.4, 0.0, 0.0, 0.0};
                dtVel = 0.005;
                int [] motorChan = {5, 4, 3, 2, 1, 0};  // Which channel to use for this motor
                int [] switchChan = {5, 4, 3, 1, 2, 0};  // Which channel to use for this motor
                Motor m0 = null;
                for(int i = 0; i < 6; i++)
                {
                    m0 = new Motor(
                            0.0, // double r0 (initial raw position)
                            scale[i], // double engrgToRawPos, counts/deg
                            scale[i], // or counts/cm for gripper
                            0.0, // double actRaw0
                            1.0, // double engrgToRawAct (duty cycle for both)
                            dtVel, // double dtVel
                            motorChan[i], // int posChan
                            motorChan[i], // int actChan
                            switchChan[i], // int switchChan
                            1 // int nSwitches
                            );
                    mtr.add(m0);
                    m0.setEngrgAct(motorActuation[i]);
                }

                if(!realMotors)
                {
                    // Only instantiate for simulated operation
                    String [] axisNames = {"WaistSim", "UpperArmSim", "ForeArmSim",
                       "WristRollSim", "WristPitchSim", "GripperSim"};
                    double [] k1Vals = {1.0 / 0.03, 1.0 / 0.015, 1.0 / 0.025,
                        1.0 / 0.026, 1.0 / 0.026, 1.0 / 0.1};
                    // The shoulder and elbow axes are affected by gravity so
                    // some values here have been changed to match particular
                    // experiments with the robot.
                    //double [] gainVals = {25.0, 13.3, 28.9, 61.9, 61.9, 6.2};
                    double [] gainVals = {25.0, 25.0, 28.9, 61.9, 61.9, 6.2};
                    double [] sw0Vals = {115.0, -40.0, -50.0, 31.9, -20.0, -1.0e10};
                    double [] sw1Vals = {130.0, 37.7, 5.3, 65.5, 10.2, 1.0e10};
                    double [] hsNegVals = {-1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10, -1.0e10};
                    double [] hsPosVals = {1.0e10, 1.0e10, 1.0e10, 1.0e10, 1.0e10, 4.0};

                    for(int i = 0; i < 6; i++)
                    {
                        m = new MotorSimScorbot2(
                                axisNames[i], //String name,
                                sys, //TrjSys sys,
                                0.0, //double u,  // PWM duty cycle
                                k1Vals[i], //double k1, // 1/time constant, 1/sec
                                gainVals[i], //double ssGain, // steady state gain
                                  // for gripper, (cm/s)/(duty cycle)
                                sw0Vals[i], //double sw0, // Bounding locations for switch
                                sw1Vals[i], //double sw1,
                                false, //boolean invertSwitch,
                                hsNegVals[i], //double hsNeg, // Hardstop locations
                                hsPosVals[i], //double hsPos,
                                1.0e1, //double khs,  // Spring constant for hardstop
                                true // boolean useAdaptiverSolver
                                );
                        mSim.add(m);
                    }
                }

                // Create PID controllers
                // See below for the definition of this "inner" class
                for(int i = 0; i < 6; i++)
                {
                    String [] axisNames = {"WaistPID", "Upper ArmPID",
                        "Fore ArmPID",
                        "Wrist RollPID", "Wrist PitchPID", "GripperPID"};
                    // Note -- the two sets of kp, kd are for simulation
                    // tuned (first) and real robot tuned (second)
                    double [] kpVals = {3.0, 4.0, 4.0, 1.0, 1.0, 10.0};
                    //double [] kpVals = {8.0, 12.0, 12.0, 3.5, 3.5, 150.0};
                    double [] kiVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    double [] kdVals = {0.0, 0.0, 0.02, 0.0, 0.0, 0.0};
                    //double [] kdVals = {0.1, 0.1, 0.1, 0.03, 0.03, 3.0};
                    double [] setVals = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                    pPID = new PosPID(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            0.005, //double dt, sec
                            -1.0, //double mMin, volts
                            1.0, //double mMax,
                            0.0, //double mOff,
                            PIDControl.PID_ON, //int initialState, start with control on
                            true, //boolean taskActive,
                            kpVals[i], //double kp,
                            kiVals[i], //double ki,
                            kdVals[i], //double kd,
                            0.0, //double integ0,
                            true, //boolean triggerMode,
                            true, //boolean useAntiWindup
                            mtr.get(i) // Motor mot
                            );

                    pPID.SetStateTracking(true);
                    pPID.SetSetpoint(setVals[i]);  // deg (cm for gripper)
                    fb.add(pPID);
                }

                // Create profile tasks
                for(int i = 0; i < 6; i++)
                {
                    String [] axisNames = {"WaistProf", "Upper ArmProf",
                        "Fore ArmProf",
                        "Wrist RollProf", "Wrist PitchProf", "GripperProf"};
                    double [] cruise = {20.0, 10.0, 20.0, 40.0, 40.0, 50.0};
                    // Waist accel reduced to 70 for smoother motion
                    double [] accel = {70.0, 500.0, 500.0, 500.0, 500.0, 20.0};
                    double [] target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    
                    mProfCosine =new MotorCosineProfileP(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            ProfileGenerator.PROFILE_START, //int initialState,
                            true, //boolean taskActive,
                            0.005, //0.001, //double dtNominal,
                            cruise[i], //double dsdtCruise
                            accel[i], //double accel,
                            accel[i], //double decel
                            fb.get(i) // SISOFeedback cntlr
                            );

                    mProfCosine.setNewProfile(0.0, target[i]);  // New profile -- s0, sE
                    mProfCosine.SetStateTracking(true);
                    mcpp.add(mProfCosine);
                }
                
                // Jog tasks
                // Make these tasks the most generic jog-only configuration
                // Other uses(eg, event detection) will require further configuration.
                // Put this here so it's in scope during control
                // Make sure there is no movement until the user commands
                // movement from the GUI
                double [] velJog = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                for(int i = 0; i < 6; i++)
                {
                    String [] axisNames = {"WaistJog", "UpperArmJog",
                        "ForeArmJog",
                        "WristRollJog", "WristPitchJog", "GripperJog"};
                    double [] sNeg = {-180.0, -180.0,-180.0,-180.0,-180.0, -10.0};
                    double [] sPos = {180.0, 180.0, 180.0, 180.0, 180.0, 10.0};

                    newJog =new NewJog(
                            axisNames[i], //String name,
                            sys, //TrjSys sys,
                            NewJog.START, //int initialState
                            true, //boolean taskActive,
                            0.005, //0.001, //double dtNominal,
                            0.0, //double s0,
                            sNeg[i], //double sNeg,
                            sPos[i], //double sPos,
                            velJog[i], //double velJog
                            fb.get(i), // SISOFeedback cntlr
                            mtr.get(i), //Motor mtr,
                            false, //boolean stopOnEvent, These values entered to thehoming task
                            false, //boolean switchLookForValue, 
                            0.0, //double errorVal, 
                            EventType.None //EventType eType
                            );
                    newJog.SetStateTracking(true);
                    newJogList.add(newJog);
                }

                // Create the 'homing' task
                dt = 1.0e-5;  // For simulated time

                tFinal = 50.0;  // Use big number here to run indefinitely
                dtLog = 2.0e-3;
                doLogging = true;
                if(doLogging)
                {
                    // Create a data logging task
                    log = new DataLoggerScorbot2
                            (
                            "DataLog1", //String name,
                            sys, //TrjSys sys,
                            0, //int initialState, (no states in this task)
                            true, //boolean taskActive,
                            dtLog, //double dt,
                            multi, //MultiMotor mm,
                            0.0, //double ts,
                            tFinal //double te
                            );
                }

                // Create the 'Homing' task
                double [] preHomeLocations = {105.0, 27.0, -5.0, 21.0, -5.0, -4.5};
                boolean [] stopOnEvent = {true, true, true, true, true, true};
                boolean [] switchLookForVal = {true, false, false, true, false, true};
                EventType [] event = {EventType.Switch, EventType.Switch, EventType.Switch, 
                		EventType.Switch, EventType.Switch, EventType.Hardstop};
                double [] jogVelHome = {2.0, 2.0, 2.0, 2.0, 2.0, 0.5};
                double [] errorVal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
                
                homing = new Homing(
                		"Homing", //String name, 
                		sys, //TrjSys sys, 
                		Homing.off, //int initialState,
                        true, //boolean taskActive, 
                        0.005, //double dtNominal, 
                        newJogList, //ArrayList<NewJog> joggerList, 
                        mtr, //ArrayList<Motor> motorList,
                        mcpp, //ArrayList<MotorCosineProfileP> mcp, 
                        preHomeLocations, //double [] preHomeLocations	
                        stopOnEvent, //boolean [] stopOnEvent
                        switchLookForVal, //boolean [] switchLookForVal
                        event, //EventType [] event 
                        jogVelHome, //double [] jogVelHome 
                        errorVal //double [] errorVal                       
                );
                //homing.SetCommand(Homing.startHoming); // For now; this will move to another task later
                homing.SetStateTracking(true);
                
                // Make a task for the GUI -- It starts out in Jogging mode
                homingGUI = new HomeOpInt(
                        "HomingGUI", //String name, 
                        sys, //TrjSys sys, 
                        HomeOpInt.Init1, //int initialState,
                        true, //boolean taskActive, *** don't run for the moment 
                        0.050, //double dtNominal (sec),
                        newJogList, //ArrayList<NewJog> joggerList,
                        mtr, //ArrayList<Motor> motorList
                        homing   // Homing homing
                        );
                homingGUI.SetStateTracking(true);
                
                
                if(realMotors)
                {
                    VisaIO.vioInit();
                }

                // Create a timing histogram
                Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);
                double tPrev = 0.0;
                boolean first = true;  // True during first run through loop
                double dtProg = 1.0, tNextProg = 0.0;

                timeKeeper.resetRunningTime();
                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    double dtLoop = tCur - tPrev;
                    if(dtLoop > 0.008)
                        System.out.println("t dtLoop " + tCur + " " + dtLoop);
                    if(!first)h.HistValue(dtLoop);  // Ignore the first pass

                    // Connect motor data to simulation or real motor
                    multi.InterfaceMotors(tCur);

                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    // Data logging has been replaced by a separate task
                    sys.IncrementRunningTime(dt);
                    first = false;
                    tPrev = tCur;

                    // Check to see if user has signalled to stop
                    if(homingGUI.getStopVal())break;
                    if(tCur >= tNextProg)
                    {
                    	tNextProg += dtProg;
                    	System.out.printf("time = %g\n", tCur);
                    }
                }
                multi.TurnOffMotors();
                h.WriteHistogram("HistPtToPt.txt");
                // Write homing information
                for(int i = 0; i < 6; i++)
                {
                	Motor mm = mtr.get(i);
                	System.out.printf("%d %b %g\n", i, mm.GetHomeSet(),
                			mm.GetHomePosition());
                }
                break;
            }
        }

        if(doLogging)
        {
            log.WriteAllData("TestData.txt");
            try
            {
                // Show the data plot
                switch(s)
                {
                    case JogTest:
                    case JogWithGUI:
                    case HomingTest:
                    
                        Runtime.getRuntime().exec("wgnuplot RobotPositions2.gnu -");
                        Runtime.getRuntime().exec("wgnuplot RobotVelocities2.gnu -");
                        Runtime.getRuntime().exec("wgnuplot RobotActuations2.gnu -");
                        Runtime.getRuntime().exec("wgnuplot RobotErrors2.gnu -");
                        break;
                        
                    case HomingWithGUI:
                    	Runtime.getRuntime().exec("wgnuplot RobotPositions2.gnu -");
                    	break;
                }
            }
            catch (IOException ex)
            {
                System.out.println("<Plotting> " + ex);
            }
        }
    }  // End of case JogWithGUI

    // Define an "inner class" for a PID position controller
    public class PosPID extends PIDControl
    {
        Motor mot;

        public PosPID(String name, TrjSys sys, double dt,
            double mMin, double mMax, double mOff,
            int initialState, boolean taskActive,
            double kp, double ki, double kd, double integ0,
            boolean triggerMode, boolean useAntiWindup, Motor mot)
        {
            super(name, sys, dt, mMin, mMax, mOff, initialState,
                    taskActive, kp, ki, kd, integ0, triggerMode,
                    useAntiWindup);
            this.mot = mot;
        }

        public PosPID CreateClone(String newName)
        {
            PosPID p = new PosPID(newName, sys, dt, mMin, mMax, mOff,
                    initialState, taskActive, kp, ki, kd, integ0, triggerMode,
                    useAntiWindup, mot);
            return p;
        }

        @Override
        public double FindProcessValue()
        {
            // Return the process value in engineering units
            return mot.getEngrgPos();
        }

        @Override
        public void PutActuationValue(double val)
        {
            // Set the actuation value in engineering units
            mot.setEngrgAct(val);  // Voltage to the motor
        }

    }

    public class MotorBoxProfile extends BoxProfile
    {
    	SISOFeedback cntlr = null;

    	public MotorBoxProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtBox, SISOFeedback cntlr)
        {
            super(name,sys,initialState, taskActive, dtNominal, dsdtBox);
        }

        @Override
        public void sToSetpoint(double t, double s)
        {
        	cntlr.SetSetpoint(s);
            cntlr.SetTrigger(true);
        }
        
        @Override
        public void SetControllerTriggerMode(boolean m)
        {
        	cntlr.SetTriggerMode(m);
        }
        
        @Override
        public double GetControllerSetpoint()
        {
        	return cntlr.GetSetpoint();
        }
    }

    public class MotorTrapezoidProfile extends TrapezoidProfile
    {
    	SISOFeedback cntlr = null;
    	
        public MotorTrapezoidProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtCruise,
            double accel, double decel, SISOFeedback cntlr)
        {
            super(name,sys,initialState, taskActive, dtNominal, dsdtCruise,
                    accel, decel);
            this.cntlr = cntlr;
        }

        @Override
        public void sToSetpoint(double t, double s)
        {
        	cntlr.SetSetpoint(s);
            cntlr.SetTrigger(true);
        }
        
        @Override
        public void SetControllerTriggerMode(boolean m)
        {
        	cntlr.SetTriggerMode(m);
        }
        
        @Override
        public double GetControllerSetpoint()
        {
        	return cntlr.GetSetpoint();
        }
    }

    // The cosine profile is used by the homing task so it has to be 
    // public (not an innner class -- it has now been moved to its own file.

    public class ScorbotJogger extends Jogger
    {
        SISOFeedback cntlr = null;

        public ScorbotJogger(String name, TrjSys sys,
            boolean taskActive, double dtNominal, double s0, double sNeg,
            double sPos, double velJog, SISOFeedback cntlr)
        {
            super(name, sys, taskActive, dtNominal, s0, sNeg, sPos, velJog);
            this.cntlr = cntlr;
        }
        @Override
        public void sToSetpoint(double t, double s)
        {
            cntlr.SetSetpoint(s);
            cntlr.SetTrigger(true);
        }
    }

    //enum EventType{Switch, Hardstop, None};  // Types of events ("None" is used
    		// where this task behaves just like an ordinary jogger)
    
    public class JogToEvent extends ScorbotJogger
    {
    	// Supports switch and hardstop events
        protected Motor mtr;
        boolean switchLookForValue = false;
        double errorVal = 0.1;
        EventType eType;
        boolean eventFound;

        public JogToEvent(String name, TrjSys sys,
            boolean taskActive, double dtNominal, double s0, double sNeg,
            double sPos, double velJog, SISOFeedback cntlr, Motor mtr,
            boolean stopOnEvent, boolean switchLookForValue, 
            double errorVal, EventType eType)
        {
            super(name, sys, taskActive, dtNominal, s0, sNeg, sPos, velJog,
                    cntlr);
            this.mtr = mtr;
            this.stopOnEvent = stopOnEvent;
            this.switchLookForValue = switchLookForValue;
            this.errorVal = errorVal;
            this.eType = eType;
            eventFound = false;
        }

        public void SetSwitchLookForValue(boolean v)
        {
            switchLookForValue = v;
        }
        
        @Override
        public boolean EventCheck(double t, double s)
        {
        	boolean rv = false;  // Return value
        	
        	switch(eType)
        	{
        	case Switch:        	
        		if(mtr.getSwitch(0) == switchLookForValue)rv = true;
        		else rv = false;
        		break;
        		
        	case Hardstop:
        		if(Math.abs(cntlr.GetError()) > errorVal)rv = true;
        		else rv = false;
        		break;
        		
        	case None:
        		rv = false;
        		break;
        		
        	default:
        		System.out.printf("<JogToEvent.eventCheck> Unknown event type");
        		System.exit(1);
        	}
        	return rv;
        }
    }

}
