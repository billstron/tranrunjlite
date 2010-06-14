package TranRunJLiteSamples;

import TranRunJLite.*;

/** A public version of this class (the 'P' on the end stands for public)
 * Previous versions of this have been inner classes.
 * @author DMAuslander. Converted to public,Jan 4, 2010
 */
public class MotorCosineProfileP extends CosineProfile
{
    SISOFeedback cntlr = null;

    public MotorCosineProfileP(String name, TrjSys sys, int initialState,
        boolean taskActive, double dtNominal, double dsdtCruise,
        double accelMax, double decelMax, SISOFeedback cntlr)
    {
        super(name,sys,initialState, taskActive, dtNominal, dsdtCruise,
                accelMax, decelMax);
        this.cntlr = cntlr;
    }

    public MotorCosineProfileP CreateClone(String newName)
    {
        MotorCosineProfileP mcp = new MotorCosineProfileP(newName, sys,
                initialState, taskActive, dtNominal, dsdtCruise,
                accelMax, decelMax, cntlr);
        return mcp;
    }

    @Override
    public void sToSetpoint(double t, double s)
    {
        cntlr.SetSetpoint(s);
        cntlr.SetTrigger(true);
    }
    
    @Override
    public void SetControllerTriggerMode(boolean m)
    {
    	cntlr.SetTriggerMode(m);
    }
    
    @Override
    public double GetControllerSetpoint()
    {
    	return cntlr.GetSetpoint();
    }
}
