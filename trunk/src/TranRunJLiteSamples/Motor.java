/* Motor Class, Motor.java
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package TranRunJLiteSamples;

/** Create a class for a motor. This class isolates the motor information
 * from the source of that information -- real or simulated
 * @author DMAuslander, 8/27/09
 */
public class Motor
{
    private double rawPosition;  // As read from the sensor
    private double prevRawPosition;
    private double prevTime;
    private double dtVel;  // How often to estimate velocity
    private boolean first;
    private double engrgToRawPos;  // Conversion factor, eg, counts/rev
    private double engrgToRawVel;  // same, for velocity
    private double rawVelocityEst; // Estimated velocity
    private double rawVelocityMeas; // Measured velocity
    private double rawActuation;  // As sent to the actuator
    private double engrgToRawAct; // Convert, eg, DA counts/volt
    private boolean [] switches;  // Homing, end-of-travel, etc.
    private int nSwitches;
    private boolean homeSet;
    private double homePosition;

    int actChan, posChan, switchChan; // Channel numbers for actuation,
            // position and homing switch channels

    public Motor(double r0, double engrgToRawPos, double engrgToRawVel,
            double actRaw0, double engrgToRawAct, double dtVel,
            int posChan, int actChan, int switchChan)
    {
        MotorInit(r0, engrgToRawPos, engrgToRawVel, actRaw0, engrgToRawAct,
                dtVel, posChan, actChan, switchChan, 0);  // Base case, no switches
    }

    // Alternate constructor with switches
    public Motor(double r0, double engrgToRawPos, double engrgToRawVel,
            double actRaw0, double engrgToRawAct, double dtVel,
            int posChan, int actChan, int switchChan, int nSwitches)
    {
        MotorInit(r0, engrgToRawPos, engrgToRawVel, actRaw0, engrgToRawAct,
                dtVel, posChan, actChan, switchChan, nSwitches);
    }

    public void MotorInit(double r0, double engrgToRawPos, double engrgToRawVel,
            double actRaw0, double engrgToRawAct, double dtVel,
            int posChan, int actChan, int switchChan, int nSwitches)
    {
        rawPosition = r0;
        this.engrgToRawPos = engrgToRawPos;
        this.engrgToRawVel = engrgToRawVel;
        rawActuation = actRaw0;
        this.engrgToRawAct = engrgToRawAct;
        this.dtVel = dtVel;
        this.posChan = posChan;
        this.actChan = actChan;
        this.switchChan = switchChan;
        this.nSwitches = nSwitches;
        if(nSwitches > 0)
            switches = new boolean[nSwitches];
        else
            switches = null;

        rawVelocityEst = 0.0;
        rawVelocityMeas = 0.0;
        prevRawPosition = rawPosition;
        homeSet = false;
        homePosition = 0.0;
        first = true;
    }

    public Motor CreateClone()
    {
        // Create a copy of this motor that has all of the same parameters
        Motor m = new Motor(rawPosition, engrgToRawPos, engrgToRawVel,
                rawActuation, engrgToRawAct, dtVel, posChan, actChan, switchChan);
        return m;
    }

    public void setRawPos(double r, double t)
    {
        rawPosition = r;
        if(first)
        {
            first = false;
            prevTime = t;
            prevRawPosition = r;
            return;
        }
        if(t <= prevTime)return;  // Time hasn't advanced, don't estimate velocity
        double dt = t - prevTime;
        if(dt >= dtVel)
        {
            // Only do velocity estimate when enough time has elapsed
            rawVelocityEst = (rawPosition - prevRawPosition) / dt;
            prevTime = t;
            prevRawPosition = rawPosition;
        }
    }

    public void setEngrgPos(double r, double t)
    {
        setRawPos(r * engrgToRawPos, t);
    }

    public void setRawVelMeas(double v)
    {
        rawVelocityMeas = v;
    }

    public void setEngrgVelMeas(double v)
    {
        setRawVelMeas(v * engrgToRawVel);
    }
    public double getEngrgPos()
    {
        return rawPosition / engrgToRawPos;
    }

    public double getRawPos()
    {
        return rawPosition;
    }

    public double getEngrgVelEst()
    {
        return rawVelocityEst / engrgToRawVel;
    }

    public double getRawVelEst()
    {
        return rawVelocityEst;
    }

    public double getEngrgVelMeas()
    {
        return rawVelocityMeas / engrgToRawVel;
    }

    public double getRawVelMeas()
    {
        return rawVelocityMeas;
    }

    public void setRawAct(double r)
    {
        rawActuation = r;
    }

    public void setEngrgAct(double r)
    {
        rawActuation = r * engrgToRawAct;
    }

    public double getEngrgAct()
    {
        return rawActuation / engrgToRawAct;
    }

    public double getRawAct()
    {
        return rawActuation;
    }

    public void setSwitch(int i, boolean v)
    {
        if(i >= nSwitches)
        {
            System.out.printf("<Motor.setSwitch> switch index out of bounds\n");
            System.out.printf("i %d, nSwitches %d\n", i, nSwitches);
            System.exit(1);
        }
        switches[i] = v;
    }

    public boolean getSwitch(int i)
    {
        if(i >= nSwitches)
        {
            System.out.printf("<Motor.getSwitch> switch index out of bounds\n");
            System.out.printf("i %d, nSwitches %d\n", i, nSwitches);
            System.exit(1);
        }
        return switches[i];
    }
    
    public void SetHomePosition(double h)
    {
    	homePosition = h;
    	homeSet = true;
    }
    
    public boolean GetHomeSet()
    {
    	return homeSet;
    }
    
    public double GetHomePosition()
    {
    	return homePosition;
    }
}
