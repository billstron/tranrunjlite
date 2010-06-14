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

/** Simulates a wathing machine or a clothes dryer, depending on the variales
 * that are passed to it.
 *
 * @author WJBurke 7/29/2009
 */
public class ClothesMachine extends TrjTask {

    double potential;  // how hard the unit mathine draws people to it.
    double kPotential;
    double pStandby;  // standby power draw
    double pRunning;  // running power draw
    double p;  // currnet power usage
    int startLoad;  // integer that indicates how many loads to run
    ClothesMachine nextUnit = null;  // ability to move to next machine
    double tState;  // length of time in current state.
    double dtCycleLength;  // cycle time of the unit in seconds
    double dt;  // how often to run the task
    double tNext;  // next time to run the task

    @Override
    public boolean RunTaskNow(TrjSys sys) {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    /** State definitions
     *   OFF state -- draws the standby power and waits for a new load
     *   RUNNING state -- draws the running power and quits based on timer.  
     */
    private enum State {

        OFF("Off State", 0),
        RUNNING("Running State", 1);
        private final String stateName;
        private final int stateNum;

        State(String stateName, int stateNum) {
            this.stateName = stateName;
            this.stateNum = stateNum;
        }

        public String stateName() {
            return this.stateName;
        }

        public int stateNum() {
            return this.stateNum;
        }
    }

    /** Start loads
     *
     */
    public void setStartLoad() {
        startLoad++;
    }

    public void setNextUnit(ClothesMachine unit) {
        nextUnit = unit;
    }

    /** get the current draw the machine has
     *
     * @return current potential
     */
    public double getPotential() {
        return potential;
    }

    public double getPower() {
        return p;
    }

    /** get the state by state number
     *
     * @param i -- state number
     * @return State value
     */
    private State getStateByNumber(int i) {
        for (State s : State.values()) {
            if (s.stateNum() == i) {
                return s;
            }
        }
        return null;
    }

    /** Constructer for the ClothesMachine class
     *
     * @param name  -- name of the task
     * @param sys  -- TrjSys that it is inside
     * @param kPotential -- the constant for the potential function
     * @param pStandby -- the standby power
     * @param pRunning -- the running power
     * @param dtCycleLength -- the lenght of time it runs.
     * @param dt -- How often to run the task
     */
    public ClothesMachine(
            String name,
            TrjSys sys,
            double kPotential,
            double pStandby,
            double pRunning,
            double dtCycleLength,
            double dt) {
        super(name, sys, State.OFF.stateNum(), true);
        stateNames.add(State.OFF.stateName());
        stateNames.add(State.RUNNING.stateName());

        this.dtCycleLength = dtCycleLength;
        this.tState = 0;
        this.kPotential = kPotential;
        this.potential = 0;
        this.pRunning = pRunning;
        this.pStandby = pStandby;
        this.p = 0;
        startLoad = 0;  // start out with no loads in the wash
        this.dt = dt;
        this.tNext = 0;
    }

    /** The RunTask does all of the work.  It operates the state machine.
     *
     * @param sys -- the TrjSys that the state belongs to.
     * @return  ???
     */
    public boolean RunTask(TrjSys sys) {
        // get the current time of the simulation. 
        double t = sys.GetRunningTime();
        // Run the state machine if it is time.  
        if (t >= tNext) {
            State state = this.getStateByNumber(currentState);
            // the state machine
            switch (state) {
                case OFF:  // the off state
                    potential += kPotential;
                    p = pStandby;
                    tState += dt;
                    // start a new load if the start load is set.  
                    if (startLoad > 0) {
                        nextState = State.RUNNING.stateNum();
                        startLoad--;
                        tState = 0;
                    }
                    break;
                case RUNNING: // the running state
                    potential = 0;
                    p = pRunning;
                    tState += dt;
                    // if it has been running long enough...
                    if (tState >= dtCycleLength) {
                        // go to the off state
                        nextState = State.OFF.stateNum();
                        tState = 0;
                        // move clothes to the next unit.  
                        if (nextUnit != null) {
                            nextUnit.setStartLoad();
                        }
                    }
                    break;
            }
            // update the state timer
            tNext += dt;
        }
        return false;
    }
}
