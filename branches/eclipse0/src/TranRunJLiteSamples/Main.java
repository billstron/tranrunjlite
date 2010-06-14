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
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/** TranRunJLite Samples
 * @author DMAuslander, July 6, 2009
 */
public class Main
{
    enum Samples{Counting, CountingDouble, MotorSim, VelocityControl,
        PositionControl, BoxProfile, TrapezoidProfile, CosineProfile};
    static final double radToDeg = 180.0 /Math.PI;
    static final double radToRev = 1.0 / (2.0 * Math.PI);
    static final double radpsToRevpm = radToRev * 60.0; // rad/sec ==> rev/min

    MotorSim ms = null;
    PosPID pPID = null;
    MotorBoxProfile mProf = null;
    MotorTrapezoidProfile mProfTrap = null;
    MotorCosineProfile mProfCosine = null;
    
    /** This package has a set of TranRunJLite sample programs
     * Each section of the switch() is a separate sample program
     * Use selection variable to decide which sample to run
     * @param args the command line arguments (not used)
     */
    public static void main(String[] args)
    {
        Main main = new Main();
        main.RunSamples();
    }

    public void RunSamples()
    {
        // Select the problem to be solved
        Samples s = Samples.TrapezoidProfile;
        TrjSys sys;
        double dt = 0.0;  // Used for samples that need a time delta
        double tFinal = 0.0;
        double dtLog = 0.0, tNextLog = 0.0;
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
            case Counting:
                sys = new TrjSys(0.0); // This sample uses simulated time

                Counting tsk = new Counting("Counting", sys, 3, 2);
                // Create the task
                tsk.SetStateTracking(true);
                dt = 1.0;  // For simulated time
                tFinal = 100.0;  // System should stop itself before this

                while(sys.GetRunningTime() <= tFinal)
                {
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    // Log data to the screen
                    System.out.printf("%g %d %d\n", sys.GetRunningTime(),
                            tsk.GetLocalCount(), tsk.GetMasterCount());
                    sys.IncrementRunningTime(dt);
                }
                break;

            case CountingDouble:
                sys = new TrjSys(0.0); // This sample uses simulated time

                Counting tsk1 = new Counting("Counting1", sys, 3, 2);
                Counting tsk2 = new Counting("Counting2", sys, 1, 3);
                // Create the task
                tsk1.SetStateTracking(true);
                tsk2.SetStateTracking(true);
                dt = 1.0;  // For simulated time
                tFinal = 100.0;  // System should stop itself before this

                while(sys.GetRunningTime() <= tFinal)
                {
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    // Log data to the screen
                    System.out.printf("%g %d %d %d %d\n", sys.GetRunningTime(),
                            tsk1.GetLocalCount(), tsk1.GetMasterCount(),
                            tsk2.GetLocalCount(), tsk2.GetMasterCount());
                    sys.IncrementRunningTime(dt);
                }
                break;

            case MotorSim:
                sys = new TrjSys(0.0); // This sample uses simulated time
                ms = new MotorSim(
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
                        0.0, //double loadB,  // friction (damping) on load, Nm/(rad/sec)
                        1.0, //double gearRatio  // unitless (gr > 1 means motor
                                //turns faster than load)
                        true // boolean useAdaptiverSolver
                        );
                dt = 1.0e-3;  // For simulated time
                tFinal = 0.2;

                while(sys.GetRunningTime() <= tFinal)
                {
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    // Log data to the screen
                    //System.out.printf("%g backEmf %g\n", sys.GetRunningTime(), m.backEmf);
                    /*
                    System.out.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                            radToRev * m.angleMotor,
                            radpsToRevpm * m.omegaMotor,
                            radToRev * m.angleLoad,
                            radpsToRevpm * m.omegaLoad);
                     */
                    // Log data to a file
                    dataFile0.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                            radToRev * ms.angleMotor,
                            radpsToRevpm * ms.omegaMotor,
                            radToRev * ms.angleLoad,
                            radpsToRevpm * ms.omegaLoad);
                    sys.IncrementRunningTime(dt);
                }
                break;

            case VelocityControl:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous sample (MotorSim)
                ms = new MotorSim(
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
                
                // Create a PID controller
                // See below for the definition of this "inner" class
                VelPID vPID = new VelPID(
                        "VelControl", //String name,
                        sys, //TrjSys sys,
                        0.001, //double dt, sec
                        -10.0, //double mMin, volts
                        10.0, //double mMax,
                        0.0, //double mOff,
                        1, //int initialState, start with control on
                        true, //boolean taskActive,
                        0.5, //double kp,
                        100.0, //double ki,
                        0.0, //double kd,
                        0.0, //double integ0,
                        false, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );

                vPID.SetSetpoint(1050.0 / radpsToRevpm);
                vPID.SetStateTracking(true);
                dt = 1.0e-4;  // For simulated time
                tFinal = 0.03;

                while(sys.GetRunningTime() <= tFinal)
                {
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    // Log data to the screen
                    //System.out.printf("%g backEmf %g\n", sys.GetRunningTime(), m.backEmf);
                    /*
                    System.out.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                            radToRev * m.angleMotor,
                            radpsToRevpm * m.omegaMotor,
                            radToRev * m.angleLoad,
                            radpsToRevpm * m.omegaLoad);
                     */
                    // Log data to a file
                    dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                            sys.GetRunningTime(),
                            radToRev * ms.angleMotor,
                            radpsToRevpm * ms.omegaMotor,
                            radToRev * ms.angleLoad,
                            radpsToRevpm * ms.omegaLoad,
                            ms.v,
                            radpsToRevpm * vPID.GetSetpoint(),
                            radpsToRevpm * vPID.GetError());
                    sys.IncrementRunningTime(dt);
                }
                break;

            case PositionControl:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous samples
                ms = new MotorSim(
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
                        0.0, //double ki,
                        0.04, //double kd,
                        0.0, //double integ0,
                        false, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );
                pPID.SetSetpoint(50.5 / radToRev);
                //pPID.SetSetpoint(1.1 / radToRev);
                pPID.SetStateTracking(true);
                dt = 5.0e-4;  // For simulated time
                tFinal = 2.0; //0.2;

                while(sys.GetRunningTime() <= tFinal)
                {
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    // Log data to the screen
                    //System.out.printf("%g backEmf %g\n", sys.GetRunningTime(), m.backEmf);
                    /*
                    System.out.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                            radToRev * m.angleMotor,
                            radpsToRevpm * m.omegaMotor,
                            radToRev * m.angleLoad,
                            radpsToRevpm * m.omegaLoad);
                     */
                    // Log data to a file
                    dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                            sys.GetRunningTime(),
                            radToRev * ms.angleMotor,
                            radpsToRevpm * ms.omegaMotor,
                            radToRev * ms.angleLoad,
                            radpsToRevpm * ms.omegaLoad,
                            ms.v,
                            radToRev * pPID.GetSetpoint(),
                            radToRev * pPID.GetError());
                    sys.IncrementRunningTime(dt);
                }
                break;

            case BoxProfile:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous samples
                ms = new MotorSim(
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
                        0.0, //double ki,
                        0.04, //double kd,
                        0.0, //double integ0,
                        true, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );

                pPID.SetStateTracking(true);

                // See below for the definition of this "inner" class
                mProf =new MotorBoxProfile(
                        "MotorProfile", //String name,
                        sys, //TrjSys sys,
                        ProfileGenerator.PROFILE_RUN, //int initialState,
                        true, //boolean taskActive,
                        0.001, //double dtNominal,
                        1500.0 / radpsToRevpm //double dsdtBox
                        );
                mProf.setNewProfile(0.0, 50.5 / radToRev);  // New profile -- s0, sE
                mProf.SetStateTracking(true);
                dt = 1.0e-5;  // For simulated time
                tFinal = 4.0; //8.0; // Use the longer profile for an up and back move
                dtLog = 5.0e-4;
                tNextLog = 0.0;
                tNextProfile = 4.0;
                nextProfileSet = false;

                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    if((tCur > tNextProfile) && !nextProfileSet)
                    {
                        nextProfileSet = true;
                        mProf.setNewProfile(50.5 / radToRev, 0.0 / radToRev);
                        mProf.SetCommand(ProfileGenerator.START_PROFILE);
                    }
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    if(tCur >= tNextLog)
                    {
                        tNextLog += dtLog;
                        // Log data to the screen
                        //System.out.printf("%g backEmf %g\n", sys.GetRunningTime(), m.backEmf);
                        /*
                        System.out.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                                radToRev * m.angleMotor,
                                radpsToRevpm * m.omegaMotor,
                                radToRev * m.angleLoad,
                                radpsToRevpm * m.omegaLoad);
                         */
                        // Log data to a file
                        dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                                sys.GetRunningTime(),
                                radToRev * ms.angleMotor,
                                radpsToRevpm * ms.omegaMotor,
                                radToRev * ms.angleLoad,
                                radpsToRevpm * ms.omegaLoad,
                                ms.v,
                                radToRev * pPID.GetSetpoint(),
                                radToRev * pPID.GetError());
                    }
                    sys.IncrementRunningTime(dt);
                }
                break;

            case TrapezoidProfile:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous samples
                ms = new MotorSim(
                        "Motor", //String name,
                        sys, //TrjSys sys,
                        0.0, //double v,  // applied voltage, volts
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
                        0.0, //double ki,
                        0.04, //double kd,
                        0.0, //double integ0,
                        true, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );

                pPID.SetStateTracking(true);

                // See below for the definition of this "inner" class
                mProfTrap =new MotorTrapezoidProfile(
                        "MotorProfile", //String name,
                        sys, //TrjSys sys,
                        ProfileGenerator.PROFILE_RUN, //int initialState,
                        true, //boolean taskActive,
                        0.001, //double dtNominal,
                        1500.0 / radpsToRevpm, //double dsdtCruise
                        2.0e3 / radpsToRevpm, //double accel,
                        2.0e3 / radpsToRevpm //double decel
                        );
                mProfTrap.setNewProfile(0.0, 50.5 / radToRev);  // New profile -- s0, sE
                mProfTrap.SetStateTracking(true);
                dt = 1.0e-5;  // For simulated time
                tFinal = 4.0; //8.0; // Use the longer profile for an up and back move
                dtLog = 5.0e-4;
                tNextLog = 0.0;
                tNextProfile = 4.0;
                nextProfileSet = false;

                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    if((tCur > tNextProfile) && !nextProfileSet)
                    {
                        nextProfileSet = true;
                        mProfTrap.setNewProfile(50.5 / radToRev, 0.0 / radToRev);
                        mProfTrap.SetCommand(ProfileGenerator.START_PROFILE);
                    }
                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status

                    if(tCur >= tNextLog)
                    {
                        tNextLog += dtLog;

                        // Log data to the screen
                        //System.out.printf("%g backEmf %g\n", sys.GetRunningTime(), m.backEmf);
                        /*
                        System.out.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                                radToRev * m.angleMotor,
                                radpsToRevpm * m.omegaMotor,
                                radToRev * m.angleLoad,
                                radpsToRevpm * m.omegaLoad);
                         */
                        // Log data to a file
                        dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                                sys.GetRunningTime(),
                                radToRev * ms.angleMotor,
                                radpsToRevpm * ms.omegaMotor,
                                radToRev * ms.angleLoad,
                                radpsToRevpm * ms.omegaLoad,
                                ms.v,
                                radToRev * pPID.GetSetpoint(),
                                radToRev * pPID.GetError());
                    }
                    sys.IncrementRunningTime(dt);
                }
                break;

            case CosineProfile:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous samples
                ms = new MotorSim(
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
                        0.0, //double ki,
                        0.04, //double kd,
                        0.0, //double integ0,
                        true, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );

                pPID.SetStateTracking(true);

                // See below for the definition of this "inner" class
                mProfCosine =new MotorCosineProfile(
                        "MotorProfile", //String name,
                        sys, //TrjSys sys,
                        ProfileGenerator.PROFILE_RUN, //int initialState,
                        true, //boolean taskActive,
                        0.001, //double dtNominal,
                        1500.0 / radpsToRevpm, //double dsdtCruise
                        2.0e3 / radpsToRevpm, //double accel,
                        2.0e3 / radpsToRevpm //double decel
                        );
                mProfCosine.setNewProfile(0.0, 50.5 / radToRev);  // New profile -- s0, sE
                mProfCosine.SetStateTracking(true);
                dt = 1.0e-5;  // For simulated time
                tFinal = 4.0; //8.0; // Use the longer profile for an up and back move
                dtLog = 5.0e-4;
                tNextLog = 0.0;
                tNextProfile = 4.0;
                nextProfileSet = false;

                while(sys.GetRunningTime() <= tFinal)
                {
                    double tCur = sys.GetRunningTime();
                    if((tCur > tNextProfile) && !nextProfileSet)
                    {
                        nextProfileSet = true;
                        mProfCosine.setNewProfile(50.5 / radToRev, 0.0 / radToRev);
                        mProfCosine.SetCommand(ProfileGenerator.START_PROFILE);
                    }

                    if(sys.RunTasks())break; // Run all of the tasks
                        // RunTasks() returns system stop status
                    if(tCur >= tNextLog)
                    {
                        tNextLog += dtLog;

                        // Log data to the screen
                        //System.out.printf("%g backEmf %g\n", sys.GetRunningTime(), m.backEmf);
                        /*
                        System.out.printf("%g %g %g %g %g\n", sys.GetRunningTime(),
                                radToRev * m.angleMotor,
                                radpsToRevpm * m.omegaMotor,
                                radToRev * m.angleLoad,
                                radpsToRevpm * m.omegaLoad);
                         */
                        // Log data to a file
                        dataFile0.printf("%g %g %g %g %g %g %g %g\n",
                                sys.GetRunningTime(),
                                radToRev * ms.angleMotor,
                                radpsToRevpm * ms.omegaMotor,
                                radToRev * ms.angleLoad,
                                radpsToRevpm * ms.omegaLoad,
                                ms.v,
                                radToRev * pPID.GetSetpoint(),
                                radToRev * pPID.GetError());
                    }
                    sys.IncrementRunningTime(dt);
                }
                break;

            }
        dataFile0.close();
        try
        {
            // Show the data plot
            switch(s)
            {
                case MotorSim:
                    Runtime.getRuntime().exec("wgnuplot MotorSimPlot.gnu -");
                    break;
                case VelocityControl:
                    Runtime.getRuntime().exec("wgnuplot VelControlPlot.gnu -");
                    break;
                case PositionControl:
                case BoxProfile:
                case TrapezoidProfile:
                case CosineProfile:
                    Runtime.getRuntime().exec("wgnuplot PosControlPlot.gnu -");
                    break;
            }
        }
        catch (IOException ex)
        {
        }
    }

    // Define an "inner class" for a PID velocity controller
    public class VelPID extends PIDControl
    {
        public VelPID(String name, TrjSys sys, double dt,
            double mMin, double mMax, double mOff,
            int initialState, boolean taskActive,
            double kp, double ki, double kd, double integ0,
            boolean triggerMode, boolean useAntiWindup)
        {
            super(name, sys, dt, mMin, mMax, mOff, initialState,
                    taskActive, kp, ki, kd, integ0, triggerMode,
                    useAntiWindup);
        }

        @Override
        public double FindProcessValue()
        {
            // The process value can be either the motor velocity
            // or the load velocity. As long as the spring is very
            // stiff either should work as long as the scaling from
            // gear train is accounted for.
            return ms.omegaLoad;
        }

        @Override
        public void PutActuationValue(double val)
        {
            ms.v = val;  // Voltage to the motor
        }

    }

    // Define an "inner class" for a PID position controller
    // Note that the only change from the velocity control class
    // is in FindProcessValue().
    public class PosPID extends PIDControl
    {
        public PosPID(String name, TrjSys sys, double dt,
            double mMin, double mMax, double mOff,
            int initialState, boolean taskActive,
            double kp, double ki, double kd, double integ0,
            boolean triggerMode, boolean useAntiWindup)
        {
            super(name, sys, dt, mMin, mMax, mOff, initialState,
                    taskActive, kp, ki, kd, integ0, triggerMode,
                    useAntiWindup);
        }

        @Override
        public double FindProcessValue()
        {
            // The process value can be either the motor position
            // or the load position. As long as the spring is very
            // stiff either should work as long as the scaling from
            // gear train is accounted for.
            return ms.angleLoad;
        }

        @Override
        public void PutActuationValue(double val)
        {
            ms.v = val;  // Voltage to the motor
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
        
        @Override
        public void SetControllerTriggerMode(boolean m)
        {
        	pPID.SetTriggerMode(m);
        }
        
        @Override
        public double GetControllerSetpoint()
        {
        	return pPID.GetSetpoint();
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
        
        @Override
        public void SetControllerTriggerMode(boolean m)
        {
        	pPID.SetTriggerMode(m);
        }
        
        @Override
        public double GetControllerSetpoint()
        {
        	return pPID.GetSetpoint();
        }
    }

    public class MotorCosineProfile extends CosineProfile
    {
        public MotorCosineProfile(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, double dsdtCruise,
            double accelMax, double decelMax)
        {
            super(name,sys,initialState, taskActive, dtNominal, dsdtCruise,
                    accelMax, decelMax);
        }

        @Override
        public void sToSetpoint(double t, double s)
        {
            pPID.SetSetpoint(s);
            pPID.SetTrigger(true);
        }
        
        @Override
        public void SetControllerTriggerMode(boolean m)
        {
        	pPID.SetTriggerMode(m);
        }
        
        @Override
        public double GetControllerSetpoint()
        {
        	return pPID.GetSetpoint();
        }        
    }
}
