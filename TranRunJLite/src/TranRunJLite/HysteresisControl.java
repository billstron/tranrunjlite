/*
 * Copyright (c) 2009, Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *  * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *  * Neither the name of the University of California, Berkeley
 * nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package TranRunJLite;

/**
 *
 * @author William Burke <billstron@gmail.com>
 */
public class HysteresisControl extends SISOFeedback {

    double hystMax, ant;
    double e;

    public HysteresisControl(String name, TrjSys sys, double dt,
            double mMin, double mMax, double mOff,
            int initialState, boolean taskActive,
            double hystMax, double ant, boolean triggerMode) {

        super(name, sys, dt, mMin, mMax, mOff, triggerMode, initialState,
                taskActive);
        this.hystMax = hystMax;
        this.ant = ant;
        this.e = 0;
    }

    @Override
    public double FindProcessValue() {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void PutActuationValue(double val) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public boolean RunTask(TrjSys sys) {
        throw new UnsupportedOperationException("Not supported yet.");
    }
}
