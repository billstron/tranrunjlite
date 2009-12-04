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
import ODEsolver.*;

/** Simulates a DC brush motor with a gearbox and load
 * Motor inductance is ignored
 * @author DMAuslander, July 10, 2009
 */
public class MotorSim extends TrjTask
{
    // Motor parameters
    protected double v;  // applied voltage, volts
    protected double r;  // coil resistance, ohms
    protected double iM; // motor current, A
    protected double rotorInertia;  // kg-m^2 (gram-cm^2 in motor spec)
    protected double rotorDamping;  // friction (damping) on motor, Nm/(rad/sec)
    protected double torqueK;  // motor torque constant, Nm/A
    protected double backEmfK; // back EMF constant, V/(rad/sec)
    protected double backEmf; // back EMF of motor, V
    protected double loadInertia;  // kg-m^2
    protected double shaftK;  // shaft rotary spring constant, Nm/rad
    protected double shaftB;  // Damping of connection, Nm/(rad/sec)
    protected double loadB;  // friction (damping) on load, Nm/(rad/sec)
    protected double gearRatio;  // unitless (gr > 1 means motor turns faster than load)
    protected double angleMotor;  // motor position, rad
    protected double omegaMotor;  // motor speed, rad/sec
    protected double angleLoad;  // load position, rad
    protected double omegaLoad;   // load speed, rad/sec
    protected double torqueSpring;  // spring torque in shaft, Nm
    protected double torqueDamp;  // friction torque in shaft, Nm
    protected double torqueMotor; // motor applied torque, Nm
    protected double tLast;  //  Time at which simulation was last run
    protected double tCur;  // Current time
    protected double lastStep;  // For adaptive ODE solvers
    protected double stepSize;
    protected double stepMin;
    protected boolean useAdaptiveSolver;

    protected MotorODE mm = null;   // Simulation object
    protected int nStates = 4;
    protected double [] x0 = new double[nStates];
    protected double [] abstol = new double[nStates];  // Absolute and relative tolerances
    protected double reltol;

    /**
     * Constructor for the MotorSim class
     * @param name
     * @param sys The TrjSys this task is part of
     * @param v Applied voltage
     * @param r Mtor coil resistance, ohm
     * @param rotorInertia kg-m^2 (gram-cm^2 in motor spec)
     * @param torqueK Motor torque constant, Nm/A
     * @param backEmfK Back EMF constant, V/(rad/sec)
     * @param loadInertia kg-m^2
     * @param shaftK Shaft rotary spring constant, Nm/rad
     * @param loadB Friction (damping) on load, Nm/(rad/sec)
     * @param gearRatio Unitless (gr > 1 means motor turns faster than load)
     * @param useAdaptiveSolver 'true' to use the adaptive ODE solver
     */
    public MotorSim(
            String name,
            TrjSys sys,
            double v,  // applied voltage, volts
            double r,  // coil resistance, ohms
            double rotorInertia,  // kg-m^2 (gram-cm^2 in motor spec)
            double rotorDamping,  // friction (damping) on motor, Nm/(rad/sec)
            double torqueK,  // motor torque constant, Nm/A
            double backEmfK, // back EMF constant, V/(rad/sec)
            double loadInertia,  // kg-m^2
            double shaftK,  // shaft rotary spring constant, Nm/rad
            double shaftB,  // Damping of connection, Nm/(rad/sec)
            double loadB,  // friction (damping) on load, Nm/(rad/sec)
            double gearRatio,  // unitless (gr > 1 means motor
                        // turns faster than load)
            boolean useAdaptiveSolver
            )
    {
        super(name, sys, 0 /*initial state*/, true /*taskActive*/);
        this.v = v;
        this.r = r;
        this.rotorInertia = rotorInertia;
        this.rotorDamping = rotorDamping;
        this.torqueK = torqueK;
        this.backEmfK = backEmfK;
        this.loadInertia = loadInertia;
        this.shaftK = shaftK;
        this.shaftB = shaftB;
        this.loadB = loadB;
        this.gearRatio = gearRatio;
        this.useAdaptiveSolver = useAdaptiveSolver;
        
        tLast = 0.0;
        // Create an ODE (simulation) object
        // State variables:
        //  velMotor, angleMotor, velLoad, angleLoad
        for(int i = 0; i < nStates; i++)
        {
            x0[i] = 0.0;  // State variable initial values
            abstol[i] = 1.e-4;  // Absolute tolerance for adaptive solvers
        }
        reltol = 1.e-4;

        mm = new MotorODE(
                nStates, //int nn,
                x0,  //double xx0[],
                0.0, //double t0,
                abstol,  //double [] abstol,
                reltol //double reltol
                );
        stepMin = 1.e-7;  // Smallest allowable adaptive step size
        stepSize = 1.e-4;  // Nominal step size
    }

    public MotorSim CreateClone(String newName)
    {
        // Create a copy of this object with all of the same parameter values
        // except for the name
        MotorSim ms = new MotorSim(newName, sys, v, r, rotorInertia, 
                rotorDamping, torqueK, backEmfK, loadInertia, shaftK, shaftB, 
                loadB, gearRatio, useAdaptiveSolver);
        return ms;
    }

    public boolean RunTaskNow(TrjSys sys)
    {
        // The simulation runs all the time and has no states.
        tCur = sys.GetRunningTime();
        if(tCur <= tLast)
        {
            // Make sure time has moved forward
            tLast = tCur;
            return false;
        }
        else return true;
    }

    @Override
    public boolean RunTask(TrjSys sys)
    {
        // The simulation runs all the time and has no states.
        // This method just moves the simulation forward to the
        // current time.
        // Run the simulation to the current time

        tCur = sys.GetRunningTime();
        if(useAdaptiveSolver)
        {
            lastStep = mm.multiStepAdaptive(tCur - tLast, stepSize, stepMin);
            stepSize = lastStep;  // For the next iteration
        }
        else
        {
            mm.multiStepFixed(tCur - tLast,stepSize);
        }
        tLast = tCur;
        return false;
    }

    // Create an inner class for the simulation
    public class MotorODE extends RKF45
    {
        public MotorODE(int nn,double xx0[],double t0,
                double [] abstol,double reltol)
        {
            super(nn,xx0,t0,abstol,reltol);
        }

        @Override
        public void deriv()
        {
            // State variables:
            // velMotor, angleMotor, velLoad, angleLoad
            omegaMotor = x[0];
            angleMotor = x[1];
            omegaLoad = x[2];
            angleLoad = x[3];
            backEmf = backEmfK * omegaMotor;
            iM = (v - backEmf) / r;  // Current through motor
            torqueMotor = torqueK * iM;  // Torque delivered by motor
            double motorFrict = rotorDamping * omegaMotor;
            // Position delta across the shaft (spring)
            double angleGear = angleMotor / gearRatio;
            double angleDelta = angleGear - angleLoad;  // Twist of shaft
            torqueSpring = shaftK * angleDelta; //Torque across the shaft
            torqueDamp = shaftB * (omegaMotor / gearRatio - omegaLoad);
            // This gives acceleration of the motor inertia
            dx[0] = (torqueMotor - motorFrict -
                    (torqueSpring / gearRatio) - (torqueDamp / gearRatio))
                    / rotorInertia;
            dx[1] = omegaMotor;  // d angleMotor / dt = omegaMotor

            // Now do the load side
            double loadDamping = loadB * omegaLoad;
            dx[2] = (torqueSpring - loadDamping) / loadInertia;
            dx[3] = omegaLoad;
        }
    }
}
