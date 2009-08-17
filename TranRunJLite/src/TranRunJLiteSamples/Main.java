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
public class Main {

    enum Samples {

        Counting, CountingDouble, MotorSim, VelocityControl,
        PositionControl, ClothesWashing
    };
    static final double radToDeg = 180.0 / Math.PI;
    static final double radToRev = 1.0 / (2.0 * Math.PI);
    static final double radpsToRevpm = radToRev * 60.0; // rad/sec ==> rev/min
    MotorSim m = null;

    /** This package has a set of TranRunJLite sample programs
     * Each section of the switch() is a separate sample program
     * Use selection variable to decide which sample to run
     * @param args the command line arguments (not used)
     */
    public static void main(String[] args) {
        Main main = new Main();
        main.RunSamples();
    }

    public void RunSamples() {
        // Select the problem to be solved
        //Samples s = Samples.MotorSim;
        Samples s = Samples.ClothesWashing;

        TrjSys sys;
        double dt = 0.0;  // Used for samples that need a time delta
        double tFinal = 0.0;

        // Set up a file for writing results
        PrintWriter dataFile0 = null;
        try {
            FileWriter fW = new FileWriter("dataFile0.txt");
            dataFile0 = new PrintWriter(fW);
        } catch (IOException e) {
            System.out.println("IO Error " + e);
            System.exit(1);  // File error -- quit
        }

        switch (s) {
            case Counting:
                sys = new TrjSys(0.0); // This sample uses simulated time

                Counting tsk = new Counting("Counting", sys, 3, 2);
                // Create the task
                tsk.SetStateTracking(true);
                dt = 1.0;  // For simulated time
                tFinal = 100.0;  // System should stop itself before this

                while (sys.GetRunningTime() <= tFinal) {
                    if (sys.RunTasks()) {
                        break; // Run all of the tasks
                    }                        // RunTasks() returns system stop status
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

                while (sys.GetRunningTime() <= tFinal) {
                    if (sys.RunTasks()) {
                        break; // Run all of the tasks
                    }                        // RunTasks() returns system stop status
                    // Log data to the screen
                    System.out.printf("%g %d %d %d %d\n", sys.GetRunningTime(),
                            tsk1.GetLocalCount(), tsk1.GetMasterCount(),
                            tsk2.GetLocalCount(), tsk2.GetMasterCount());
                    sys.IncrementRunningTime(dt);
                }
                break;

            case MotorSim:
                sys = new TrjSys(0.0); // This sample uses simulated time
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
                        (1.0 / 394.0) * (60.0 / (2.0 * Math.PI)), //double backEmfK,
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

                while (sys.GetRunningTime() <= tFinal) {
                    if (sys.RunTasks()) {
                        break; // Run all of the tasks
                    }                        // RunTasks() returns system stop status
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
                            radToRev * m.angleMotor,
                            radpsToRevpm * m.omegaMotor,
                            radToRev * m.angleLoad,
                            radpsToRevpm * m.omegaLoad);
                    sys.IncrementRunningTime(dt);
                }
                break;

            case VelocityControl:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous sample (MotorSim)
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
                        (1.0 / 394.0) * (60.0 / (2.0 * Math.PI)), //double backEmfK,
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
                        0.1, //double ki,
                        0.0, //double kd,
                        0.0, //double integ0,
                        false, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );

                vPID.SetSetpoint(1050.0 / radpsToRevpm);
                vPID.SetStateTracking(true);
                dt = 1.0e-4;  // For simulated time
                tFinal = 0.03;

                while (sys.GetRunningTime() <= tFinal) {
                    if (sys.RunTasks()) {
                        break; // Run all of the tasks
                    }                        // RunTasks() returns system stop status
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
                            radToRev * m.angleMotor,
                            radpsToRevpm * m.omegaMotor,
                            radToRev * m.angleLoad,
                            radpsToRevpm * m.omegaLoad,
                            m.v,
                            radpsToRevpm * vPID.GetSetpoint(),
                            radpsToRevpm * vPID.GetError());
                    sys.IncrementRunningTime(dt);
                }
                break;

            case PositionControl:
                sys = new TrjSys(0.0); // This sample uses simulated time
                // Use the same motor used in the previous samples
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
                        (1.0 / 394.0) * (60.0 / (2.0 * Math.PI)), //double backEmfK,
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
                PosPID pPID = new PosPID(
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
                        40.0, //double kd,
                        0.0, //double integ0,
                        false, //boolean triggerMode,
                        true //boolean useAntiWindup
                        );

                pPID.SetSetpoint(1.1 / radToRev);
                pPID.SetStateTracking(true);
                dt = 5.0e-4;  // For simulated time
                tFinal = 0.2;

                while (sys.GetRunningTime() <= tFinal) {
                    if (sys.RunTasks()) {
                        break; // Run all of the tasks
                    }                        // RunTasks() returns system stop status
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
                            radToRev * m.angleMotor,
                            radpsToRevpm * m.omegaMotor,
                            radToRev * m.angleLoad,
                            radpsToRevpm * m.omegaLoad,
                            m.v,
                            radToRev * pPID.GetSetpoint(),
                            radToRev * pPID.GetError());
                    sys.IncrementRunningTime(dt);
                }
                break;
            case ClothesWashing:
                sys = new TrjSys(0.0);
                ClothesMachine dryer = new ClothesMachine("Dryer", sys, 0.1,
                        5, 1000, 60 * 60, 20);
                ClothesMachine washer = new ClothesMachine("Washer", sys, 0.1,
                        10, 300, 45 * 60, 20);
                washer.setNextUnit(dryer);
                washer.setStartLoad();
                dt = 10;
                tFinal = 3 * 60 * 60;
                while (sys.GetRunningTime() <= tFinal) {
                    if (sys.RunTasks()) {
                        break; // Run all of the tasks
                    } // RunTasks() returns system stop status
                    // Log data to a file
                    dataFile0.println(sys.GetRunningTime() + "\t"
                            + washer.getPower() + "\t" + dryer.getPower());
                    sys.IncrementRunningTime(dt);
                }
                break;

        }
        dataFile0.close();
        try {
            // Show the data plot
            switch (s) {
                case MotorSim:
                    Runtime.getRuntime().exec("wgnuplot MotorSimPlot.gnu -");
                    break;
                case VelocityControl:
                    Runtime.getRuntime().exec("wgnuplot VelControlPlot.gnu -");
                    break;
                case PositionControl:
                    Runtime.getRuntime().exec("wgnuplot PosControlPlot.gnu -");
                    break;
            }
        } catch (IOException ex) {
        }
    }

    // Define an "inner class" for a PID velocity controller
    public class VelPID extends PIDControl {

        public VelPID(String name, TrjSys sys, double dt,
                double mMin, double mMax, double mOff,
                int initialState, boolean taskActive,
                double kp, double ki, double kd, double integ0,
                boolean triggerMode, boolean useAntiWindup) {
            super(name, sys, dt, mMin, mMax, mOff, initialState,
                    taskActive, kp, ki, kd, integ0, triggerMode,
                    useAntiWindup);
        }

        @Override
        public double FindProcessValue() {
            // The process value can be either the motor velocity
            // or the load velocity. As long as the spring is very
            // stiff either should work as long as the scaling from
            // gear train is accounted for.
            return m.omegaLoad;
        }

        @Override
        public void PutActuationValue(double val) {
            m.v = val;  // Voltage to the motor
        }
    }

    // Define an "inner class" for a PID position controller
    // Note that the only change from the velocity control class
    // is in FindProcessValue().
    public class PosPID extends PIDControl {

        public PosPID(String name, TrjSys sys, double dt,
                double mMin, double mMax, double mOff,
                int initialState, boolean taskActive,
                double kp, double ki, double kd, double integ0,
                boolean triggerMode, boolean useAntiWindup) {
            super(name, sys, dt, mMin, mMax, mOff, initialState,
                    taskActive, kp, ki, kd, integ0, triggerMode,
                    useAntiWindup);
        }

        @Override
        public double FindProcessValue() {
            // The process value can be either the motor position
            // or the load position. As long as the spring is very
            // stiff either should work as long as the scaling from
            // gear train is accounted for.
            return m.angleLoad;
        }

        @Override
        public void PutActuationValue(double val) {
            m.v = val;  // Voltage to the motor
        }
    }
}


