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

/** Simulates the motor of a Scorbot robot. A simple 2nd order
 * model is used because we have no data for these motors or for
 * the drive train connected to them.
 * @author DMAuslander, July 10, 2009 (original), modified by DMA
 */
public class MotorSimScorbot extends TrjTask
{
    // Motor parameters
    protected double u;  // applied input, PWM duty cycle (+/-1.0)
    protected double vel;  // joint velocity (deg/s, gripper: cm/s)
    protected double pos;  // joint angle (deg, gripper: cm)
    protected double k1, k2;  // parameters, dv/dt = -k1 v + k2 u
    protected double tLast;  //  Time at which simulation was last run
    protected double tCur;  // Current time
    protected double lastStep;  // For adaptive ODE solvers
    protected double stepSize;
    protected double stepMin;
    protected boolean useAdaptiveSolver;

    protected MotorODE mm = null;   // Simulation object
    protected int nStates = 2;
    protected double [] x0 = new double[nStates];
    protected double [] abstol = new double[nStates];  // Absolute and relative tolerances
    protected double reltol;

    /**
     * Constructor for the MotorSim class
     */
    public MotorSimScorbot(
            String name,
            TrjSys sys,
            double u,  // applied voltage, volts
            double k1, // ODE parameters
            double ssGain, // steady state gain
            boolean useAdaptiveSolver
            )
    {
        super(name, sys, 0 /*initial state*/, true /*taskActive*/);
        this.u = u;
        this.k1 = k1;
        // ODE is d vel/dt = -k1 * vel + k2 * u;
        // Steady-state gain is k2/k1
        k2 = k1 * ssGain;
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

    public MotorSimScorbot CreateClone(String newName)
    {
        // Create a copy of this object with all of the same parameter values
        // except for the name
        MotorSimScorbot ms = new MotorSimScorbot(newName, sys, u, k1, k2 / k1,
                useAdaptiveSolver);
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
            // vel,pos
            // ODE is: d(pos)/dt = vel
            // d(vel)/dt = -k1 vel + k2 u
            vel = x[0];
            pos = x[1];
            dx[0] = -k1 * vel + k2 * u;
            dx[1] = vel;
        }
    }
}
