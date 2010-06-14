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

/** Data logging task for multi-motor control
 *
 * @author DMAuslander Sept 16, 2009
 */
public class DataLoggerScorbot2 extends TrjTask
{
    double ts, te; // Time to start, end logging
    double [][] data = null;
    int nData = 0;  // Number of data rows collected
    int nColPerMotor = 8;
    ScorbotAxes2 mm = null; // Where to get the data

    public DataLoggerScorbot2(String name, TrjSys sys, int initialState,
            boolean taskActive, double dt, ScorbotAxes2 mm,
            double ts, double te)
    {
        super(name, sys, initialState, taskActive);
        this.dtNominal = dt;
        this.mm = mm;
        this.ts = ts;
        this.te = te;
        // Allocate space for data
        int nMotors = mm.NumberOfMotors();
        int nDataMax = (int)((te - ts) / dt) + 5;
        data = new double[nDataMax + 2][nMotors * nColPerMotor + 1];
        nData = 0;  // Number of data rows actually collected
    }

    @Override
    public boolean RunTask(TrjSys sys)
    {
        // Runs only when sampling is needed
        // Fill one row of the data array
        double t = sys.GetRunningTime();

        mm.FillDataRow(t, data[nData]);
        nData++;
        return false;
    }

    public void WriteAllData(String fName)
    {
        mm.WriteAllData(fName, data, nData);
    }
    
    @Override
    public boolean RunTaskNow(TrjSys sys)
    {
        double t = sys.GetRunningTime();
        if((t < ts) || (t > te))return false; // Not in data window
        return CheckTime(t);
    }

}
