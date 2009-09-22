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
import visaio.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import PerfTimerPkg.*;

/** TranRunJLite Real Motor Samples
 * @author DMAuslander, Aug. 24, 2009
 */
public class RealMotorSamples
{
    enum Samples{SingleMotorOpenLoop, SingleMotorCosine, TwoMotorCosine};
    static final double degPerRad = 180.0 /Math.PI;
    static final double radPerDeg = 1.0 / degPerRad;
    static final double radPerRev = 2.0 * Math.PI;
    static final double revPerRad = 1.0 / radPerRev;
    static final double radPsPerRevPM = radPerRev / 60.0;
    static final double revPmPerRadPs = 1.0 / radPsPerRevPM;

    boolean realMotors = false;  // Set operating mode
    boolean realTime = false;
    
    MotorSim m = null;
    PosPID pPID = null;
    MotorCosineProfile mProfCosine = null;
    DataLogger log = null;

    // To collect information (ArrayLists are more convenient to use than
    // arrays, although they are probably less efficient).
    ArrayList<Motor> mtr = new ArrayList<Motor>();
    ArrayList<MotorSim> mSim = new ArrayList<MotorSim>();
    ArrayList<SISOFeedback> fb = new ArrayList<SISOFeedback>();
    ArrayList<ProfileGenerator> prof = new ArrayList<ProfileGenerator>();

    /** This package has a set of TranRunJLite sample programs
     * Each section of the switch() is a separate sample program
     * Use selection variable to decide which sample to run
     * @param args the command line arguments (not used)
     */
    public static void main(String[] args)
    {
        RealMotorSamples main = new RealMotorSamples();
        main.RunSamples();
    }

    public void RunSamples()
    {
        // Select the problem to be solved
        Samples s = Samples.TwoMotorCosine;
        TrjSys sys;
        double dt = 0.0;  // Used for samples that need a time delta
        double tFinal = 0.0;
        double dtLog = 0.0, tNextLog = 0.0, dtVel = 0.0;
        double tNextProfile = 4.0;
        boolean nextProfileSet = false;

        // Set up a file for writing results
        PrintWriter dataFile0 = null;
        try
        {
            FileWriter fW = new FileWriter ( "dataFile0.txt" );
            dataFile0 = new PrintWriter ( fW );
        }
        catch(IOException e)
        {
            System.out.println("IO Error " + e);
            System.exit(1);  // File error -- quit
        }

        switch(s)
        {
            case SingleMotorOpenLoop:
            {
                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   rad            counts       rev
                //   rad/s          counts/s     rev/min (RPM)
                //     Actuation
                //   volts          duty cycle   duty cycle

                double countsPerRev = 400.0;  // Encoder resolution
                double pwmPerVolt = 0.1; // 10 volts converts to 1.0 duty cycle
                realMotors = false;  // Set for operating mode
                realTime = false;
                
                sys = new TrjSys(0.0); // Instantiate a "system" object

                dtVel = 0.001;
                int motorChan = 7;  // Which channel to use for this motor
                Motor m0 = new Motor(
                        0.0, // double r0 (initial raw position)
                        countsPerRev / radPerRev, // double engrgToRawPos
                        countsPerRev / radPerRev, // double engrgToRawVel
                        0.0, // double actRaw0
                        pwmPerVolt, // double engrgToRawAct
                        dtVel, // double dtVel
                        motorChan, // int posChan
                        motorChan // int actChan
                        );
                mtr.add(m0);
                double motorActuation = 5.0;  // Actuation step size, volts
                m0.setEngrgAct(motorActuation);

                if(!realMotors)
                {
                    // Only instantiate for simulated operation
                    m = new MotorSim(
                            "Motor", //String name,
                            sys, //TrjSys sys,
                            motorActuation, //double v,  // applied voltage, volts
                            0.5*2.23, //double r,  // coil resistance, ohms
                            20.0e-7, //double rotorInertia,
                                // kg-m^2 (gram-cm^2 in motor spec; 10^-7 conversion)
                            0.0, //2.0e-2, //double rotorDamping,
                                    // friction (damping) on motor, Nm/(rad/sec)
                            0.5*24.2e-3, //double torqueK,  // motor torque constant, Nm/A
                            0.5*(1.0/394.0)*(60.0/(2.0*Math.PI)), //double backEmfK,
                                    // back EMF constant, V/(rad/sec)
                            10.0e-7, //double loadInertia,  // kg-m^2
                            0.5e3, //double shaftK,  // shaft rotary spring constant, Nm/rad
                            1.e-1, //double shaftB;  // Damping of connection, Nm/(rad/sec)
                            0.0, //double loadB,  // friction (damping) on load, Nm/(rad/sec)
                            1.0, //double gearRatio  // unitless (gr > 1 means motor
                                    //turns faster than load)
                            true // boolean useAdaptiverSolver
                            );
                }
                if(realMotors)dt = 1.8e-6;  // Calibrated time
                else dt = 1.0e-4;  // For simulated time
                tFinal = 0.1;
                dtLog = 0.2e-3;
                tNextLog = 0.0;

                if(realMotors)
                {
                    VisaIO.vioInit();
                }

                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    // Connect motor data to simulation or real motor
                    if(realMotors)
                    {
                        // Real
                        m0.setRawPos(-VisaIO.vioGetMotorPosition(m0.posChan, 1.0),
                                tCur);
                        VisaIO.vioSetMotorActuation(m0.actChan, m0.getRawAct());
                    }
                    else
                    {
                        // Simulation
                        m0.setEngrgPos(m.angleMotor, tCur);
                        m.v = m0.getEngrgAct();
                    }

                    if(tCur >= tNextLog)
                    {
                        tNextLog += dtLog;
                        // Log data to a file
                        if(realMotors)
                        {
                            dataFile0.printf("%g %g %g %g %g\n",
                                    sys.GetRunningTime(),
                                    m0.getEngrgPos() * revPerRad,
                                    0.0, //radpsToRevpm * m.omegaMotor,
                                    m0.getRawPos(),
                                    m0.getEngrgVelEst() * revPmPerRadPs);
                        }
                        else
                        {
                            dataFile0.printf("%g %g %g %g %g\n",
                                    sys.GetRunningTime(),
                                    revPerRad * m.angleMotor,
                                    revPmPerRadPs * m.omegaMotor,
                                    revPerRad * m.angleLoad,
                                    m0.getEngrgVelEst() * revPmPerRadPs);
                        }
                    }
                    sys.IncrementRunningTime(dt);
                }
                if(realMotors)
                {
                    // Turn off actuation
                    VisaIO.vioSetMotorActuation(motorChan, 0.0);
                }
                break;
            }

            case SingleMotorCosine:
            {
                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   rad            counts       rev
                //   rad/s          counts/s     rev/min (RPM)
                //     Actuation
                //   volts          duty cycle   duty cycle
                
                double countsPerRev = 400.0;  // Encoder resolution
                double pwmPerVolt = 0.1; // 10 volts converts to 1.0 duty cycle
                realMotors = false;  // Set for operating mode
                realTime = false;

                sys = new TrjSys(0.0); // This sample uses simulated time

                dtVel = 0.01;
                int motorChan = 7;  // Which channel to use for this motor
                Motor m0 = new Motor(
                        0.0, // double r0 (initial raw position)
                        countsPerRev * revPerRad, // double engrgToRawPos
                        countsPerRev * revPerRad, // double engrgToRawVel
                        0.0, // double actRaw0
                        pwmPerVolt, // double engrgToRawAct
                        dtVel, // double dtVel
                        motorChan, // int posChan
                        motorChan // int actChan
                        );
                mtr.add(m0);

                // Try to match the actual motor as well as possible
                if(!realMotors)
                {
                    // Only create the simulation object when running in
                    // simulation mode
                    m = new MotorSim(
                            "Motor", //String name,
                            sys, //TrjSys sys,
                            2.0, //double v,  // applied voltage, volts
                            2.23, //double r,  // coil resistance, ohms
                            41.7e-7, //double rotorInertia,
                                // kg-m^2 (gram-cm^2 in motor spec; 10^-7 conversion)
                            0.0, //2.0e-2, //double rotorDamping,
                                    // friction (damping) on motor, Nm/(rad/sec)
                            24.2e-3, //double torqueK,  // motor torque constant, Nm/A
                            (1.0/394.0)*(60.0/(2.0*Math.PI)), //double backEmfK,
                                    // back EMF constant, V/(rad/sec)
                            50.0e-7, //double loadInertia,  // kg-m^2
                            0.5e3, //double shaftK,  // shaft rotary spring constant, Nm/rad
                            1.e-1, //double shaftB;  // Damping of connection, Nm/(rad/sec)
                            0.0, //5.0e-4, //double loadB,  // friction (damping) on load, Nm/(rad/sec)
                            1.0, //double gearRatio  // unitless (gr > 1 means motor
                                    //turns faster than load)
                            true // boolean useAdaptiverSolver
                            );
                }

                // Create a PID controller
                // See below for the definition of this "inner" class
                pPID = new PosPID(
                        "PosControl", //String name,
                        sys, //TrjSys sys,
                        0.001, //double dt, sec
                        -10.0, //double mMin, volts
                        10.0, //double mMax,
                        0.0, //double mOff,
                        1, //int initialState, start with control on
                        true, //boolean taskActive,
                        3.0, //double kp,
                        20.0, //double ki,
                        0.03, //0.04, //double kd,
                        0.0, //double integ0,
                        true, //boolean triggerMode,
                        true, //boolean useAntiWindup
                        m0 // Motor mot
                        );

                pPID.SetStateTracking(true);

                // See below for the definition of this "inner" class
                mProfCosine =new MotorCosineProfile(
                        "MotorProfile", //String name,
                        sys, //TrjSys sys,
                        ProfileGenerator.PROFILE_RUN, //int initialState,
                        true, //boolean taskActive,
                        0.005, //0.001, //double dtNominal,
                        1500.0 * radPsPerRevPM, //double dsdtCruise
                        2.0e3 * radPsPerRevPM, //double accel,
                        2.0e3 * radPsPerRevPM, //double decel
                        pPID // SISOFeedback cntlr
                        );
                mProfCosine.setNewProfile(0.0, 50.5 * radPerRev);  // New profile -- s0, sE
                mProfCosine.SetStateTracking(true);

                if(realMotors)dt = 2.0e-6;  // Calibrated time
                else dt = 1.0e-5;  // For simulated time
                
                tFinal = 4.0; //8.0; // Use the longer profile for an up and back move
                dtLog = 2.0e-3;
                tNextLog = 0.0;
                tNextProfile = 4.0;
                nextProfileSet = false;

                if(realMotors)
                {
                    VisaIO.vioInit();
                }
                
                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    if((tCur > tNextProfile) && !nextProfileSet)
                    {
                        nextProfileSet = true;
                        mProfCosine.setNewProfile(50.5 * radPerRev, 0.0 * radPerRev);
                        mProfCosine.SetCommand(ProfileGenerator.START_PROFILE);
                    }

                    // Connect motor data to simulation or real motor
                    if(realMotors)
                    {
                        // Real
                        m0.setRawPos(-VisaIO.vioGetMotorPosition(motorChan, 1.0),
                                tCur);
                        VisaIO.vioSetMotorActuation(motorChan, m0.getRawAct());
                    }
                    else
                    {
                        // Simulation
                        m0.setEngrgPos(m.angleMotor, tCur);
                        m.v = m0.getEngrgAct();
                    }

                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    if(tCur >= tNextLog)
                    {
                        tNextLog += dtLog;

                        // Log data to a file
                        if(realMotors)
                        {
                            dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                                    sys.GetRunningTime(),
                                    m0.getEngrgPos() * revPerRad,
                                    0.0, //radpsToRevpm * m.omegaMotor,
                                    m0.getRawPos(),
                                    m0.getEngrgVelEst() * revPmPerRadPs,
                                    m0.getRawAct(),
                                    revPerRad * pPID.GetSetpoint(),
                                    revPerRad * pPID.GetError());
                        }
                        else
                        {
                            dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                                    sys.GetRunningTime(),
                                    revPerRad * m.angleMotor,
                                    revPmPerRadPs * m.omegaMotor,
                                    revPerRad * m.angleLoad,
                                    m0.getEngrgVelEst() * revPmPerRadPs,
                                    m0.getRawAct(),
                                    revPerRad * pPID.GetSetpoint(),
                                    revPerRad * pPID.GetError());
                        }
                    }
                    sys.IncrementRunningTime(dt);
                }
                if(realMotors)
                {
                    // Turn off actuation
                    VisaIO.vioSetMotorActuation(motorChan, 0.0);
                }                
                break;
            }

            case TwoMotorCosine:
            {
                // Two identical bare motors

                // Units for this example:
                // Engineering     Raw          Display
                //     Position, velocity
                //   rad            counts       rev
                //   rad/s          counts/s     rev/min (RPM)
                //     Actuation
                //   volts          duty cycle   duty cycle

                double countsPerRev = 400.0;  // Encoder resolution
                double pwmPerVolt = 0.1; // 10 volts converts to 1.0 duty cycle
                realMotors = false;  // Set for operating mode
                realTime = false;

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
                
                sys = new TrjSys(timeKeeper); // This sample uses simulated time
                MultiMotor multi = new MultiMotor(sys, mtr, mSim, fb, prof,
                        realMotors);

                dtVel = 0.01;
                int motorChan = 7;  // Which channel to use for this motor
                Motor m0 = new Motor(
                        0.0, // double r0 (initial raw position)
                        countsPerRev * revPerRad, // double engrgToRawPos
                        countsPerRev * revPerRad, // double engrgToRawVel
                        0.0, // double actRaw0
                        pwmPerVolt, // double engrgToRawAct
                        dtVel, // double dtVel
                        motorChan, // int posChan
                        motorChan // int actChan
                        );
                mtr.add(m0);

                // Try to match the actual motor as well as possible
                if(!realMotors)
                {
                    // Only create the simulation object when running in
                    // simulation mode
                    m = new MotorSim(
                            "Motor", //String name,
                            sys, //TrjSys sys,
                            2.0, //double v,  // applied voltage, volts
                            2.23, //double r,  // coil resistance, ohms
                            41.7e-7, //double rotorInertia,
                                // kg-m^2 (gram-cm^2 in motor spec; 10^-7 conversion)
                            0.0, //2.0e-2, //double rotorDamping,
                                    // friction (damping) on motor, Nm/(rad/sec)
                            24.2e-3, //double torqueK,  // motor torque constant, Nm/A
                            (1.0/394.0)*(60.0/(2.0*Math.PI)), //double backEmfK,
                                    // back EMF constant, V/(rad/sec)
                            50.0e-7, //double loadInertia,  // kg-m^2
                            0.5e3, //double shaftK,  // shaft rotary spring constant, Nm/rad
                            1.e-1, //double shaftB;  // Damping of connection, Nm/(rad/sec)
                            0.0, //5.0e-4, //double loadB,  // friction (damping) on load, Nm/(rad/sec)
                            1.0, //double gearRatio  // unitless (gr > 1 means motor
                                    //turns faster than load)
                            true // boolean useAdaptiverSolver
                            );
                    mSim.add(m);
                }

                // Create a PID controller
                // See below for the definition of this "inner" class
                pPID = new PosPID(
                        "PosControl", //String name,
                        sys, //TrjSys sys,
                        0.001, //double dt, sec
                        -10.0, //double mMin, volts
                        10.0, //double mMax,
                        0.0, //double mOff,
                        1, //int initialState, start with control on
                        true, //boolean taskActive,
                        3.0, //double kp,
                        20.0, //double ki,
                        0.03, //0.04, //double kd,
                        0.0, //double integ0,
                        true, //boolean triggerMode,
                        true, //boolean useAntiWindup
                        m0 // Motor mot
                        );

                pPID.SetStateTracking(true);
                fb.add(pPID);

                // See below for the definition of this "inner" class
                mProfCosine =new MotorCosineProfile(
                        "MotorProfile", //String name,
                        sys, //TrjSys sys,
                        ProfileGenerator.PROFILE_RUN, //int initialState,
                        true, //boolean taskActive,
                        0.005, //0.001, //double dtNominal,
                        1500.0 * radPsPerRevPM, //double dsdtCruise
                        2.0e3 * radPsPerRevPM, //double accel,
                        2.0e3 * radPsPerRevPM, //double decel
                        pPID // SISOFeedback cntlr
                        );

                mProfCosine.setNewProfile(0.0, 50.5 * radPerRev);  // New profile -- s0, sE
                mProfCosine.SetStateTracking(true);
                prof.add(mProfCosine);

                if(realMotors)dt = 2.0e-6;  // Calibrated time
                else dt = 1.0e-5;  // For simulated time

                tFinal = 4.0; //8.0; // Use the longer profile for an up and back move
                dtLog = 2.0e-3;
                tNextLog = 0.0;
                // Create a data logging task
                log = new DataLogger
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
                tNextProfile = 4.0;
                nextProfileSet = false;

                if(realMotors)
                {
                    VisaIO.vioInit();
                }

                // Create a timing histogram
                Histogram h = new Histogram(1, 15, 1.0e-6, 2.0);
                double tPrev = 0.0;
                boolean first = true;  // True during first run through loop

                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    double dtLoop = tCur - tPrev;
                    if(dtLoop > 0.008)
                        System.out.println("t dtLoop " + tCur + " " + dtLoop);
                    if(!first)h.HistValue(dtLoop);  // Ignore the first pass
                    
                    if((tCur > tNextProfile) && !nextProfileSet)
                    {
                        nextProfileSet = true;
                        mProfCosine.setNewProfile(50.5 * radPerRev, 0.0 * radPerRev);
                        mProfCosine.SetCommand(ProfileGenerator.START_PROFILE);
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
                h.WriteHistogram(null);
                break;
            }

        }
        dataFile0.close();  // Data file is opened for all samples so close it
        log.WriteAllData("TestData.txt");
        try
        {
            // Show the data plot
            switch(s)
            {
                case SingleMotorOpenLoop:
                    if(realMotors)
                    {
                        Runtime.getRuntime().exec("wgnuplot MotorOpenLoopReal.gnu -");
                    }
                    else
                    {
                        Runtime.getRuntime().exec("wgnuplot MotorOpenLoopSim.gnu -");
                    }
                    break;

                case SingleMotorCosine:
                case TwoMotorCosine:
                    Runtime.getRuntime().exec("wgnuplot PtToPtPlotOneMotor.gnu -");
                    break;
            }
        }
        catch (IOException ex)
        {
        }
    }

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
        public MotorBoxProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtBox)
        {
            super(name,sys,initialState, taskActive, dtNominal, dsdtBox);
        }

        @Override
        public void sToSetpoint(double t, double s)
        {
            pPID.SetSetpoint(s);
            pPID.SetTrigger(true);
        }
    }

    public class MotorTrapezoidProfile extends TrapezoidProfile
    {
        public MotorTrapezoidProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtCruise,
            double accel, double decel)
        {
            super(name,sys,initialState, taskActive, dtNominal, dsdtCruise,
                    accel, decel);
        }

        @Override
        public void sToSetpoint(double t, double s)
        {
            pPID.SetSetpoint(s);
            pPID.SetTrigger(true);
        }
    }

    public class MotorCosineProfile extends CosineProfile
    {
        SISOFeedback cntlr = null;

        public MotorCosineProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtCruise,
            double accelMax, double decelMax, SISOFeedback cntlr)
        {
            super(name,sys,initialState, taskActive, dtNominal, dsdtCruise,
                    accelMax, decelMax);
            this.cntlr = cntlr;
        }

        @Override
        public void sToSetpoint(double t, double s)
        {
            cntlr.SetSetpoint(s);
            cntlr.SetTrigger(true);
            //pPID.SetSetpoint(s);
            //pPID.SetTrigger(true);
        }
    }
}
