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

/** Implements a one-task system with three states that do nothing but count
 * The purpose of this sample is to test the basic functions of TranRunJLite
 * @author DM Auslander, July 6, 2009
 */
public class Counting extends TrjTask {

    int masterCount;
    int localCount = 0;
    int max1, max2;

    public int GetLocalCount() {
        return localCount;
    }

    public int GetMasterCount() {
        return masterCount;
    }

    public Counting(String name, TrjSys sys, int max1, int max2) {
        super(name, sys, 0 /*initial state*/, true /*taskActive*/);
        masterCount = 5;
        this.max1 = max1;
        this.max2 = max2;
        // State names (must be in numerical order
        stateNames.add("Base");  // 0
        stateNames.add("Count1");  // 1
        stateNames.add("Count2");  // 2
    }
    // States
    final int baseState = 0;
    final int count1State = 1;
    final int count2State = 2;

    /** This is the functioning version of RunTask()
     * All of the state code goes here
     * @param sys The system of which this task is a part
     * @return repeatTask - if true, run the task again immediately
     * State machine:
     * State# Name    Transitions
     *   0    Base    ->Count1 if masterCount > 0
     *                ->exit otherwise
     *   1    Count1  ->Count2 if localCount <= 0
     *   2    Count2  ->Base if localCount >= 0
     */
    public boolean RunTask(TrjSys sys) {
        switch (currentState) {
            case baseState: // Base state (starts here)
                // No entry code
                // Action section
                masterCount--;  // Decrement master count

                // Transition tests
                if (masterCount <= 0) {
                    sys.SetStop();
                }
                // Otherwise
                nextState = count1State;
                break;

            case count1State: // Count1
                // Entry
                if (runEntry) {
                    localCount = max1;
                }

                // Action
                localCount--;

                // Transition test
                if (localCount <= 0) {
                    nextState = count2State;
                }
                break;

            case count2State:
                // Entry
                if (runEntry) {
                    localCount = max2;
                }

                // Action
                localCount--;

                // Tranistion test
                if (localCount <= 0) {
                    nextState = baseState;
                }
                break;

            default:
                System.out.printf("Counting Task: illegal state - %d\n",
                        currentState);
                System.exit(1);
        }
        return false;
    }
}
