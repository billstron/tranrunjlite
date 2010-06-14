package TranRunJLiteSamples;

import TranRunJLite.*;

/** A jogging class with event detection
 * This class must be a stand alone public class because it is used
 * by other classes (ie,it can't be  an inner class).
 * @author DMAuslander, Jan 2, 2010
 */
public class NewJog extends Jogger2
{
	// Supports switch and hardstop events
    SISOFeedback cntlr = null;
    protected Motor mtr;
    boolean switchLookForValue;
    double errorVal = 0.1;
    EventType eType;
    boolean eventFound;

    public NewJog(String name, TrjSys sys, int initialState,
        boolean taskActive, double dtNominal, double s0, double sNeg,
        double sPos, double velJog, SISOFeedback cntlr, Motor mtr,
        boolean stopOnEvent, boolean switchLookForValue, 
        double errorVal, EventType eType)
    {
        super(name, sys, initialState, taskActive, dtNominal, s0, sNeg, sPos, velJog);
        this.cntlr = cntlr;
        this.mtr = mtr;
        this.stopOnEvent = stopOnEvent;
        this.switchLookForValue = switchLookForValue;
        this.errorVal = errorVal;
        this.eType = eType;
        eventFound = false;
    }

    public void SetEventParameters(boolean stopOnEvent, boolean switchLookForValue,
    		double errorVal, EventType eType, double velJog)
    {
        this.stopOnEvent = stopOnEvent;
        this.switchLookForValue = switchLookForValue;
        this.errorVal = errorVal;
        this.eType = eType;    	
        this.velJog = velJog;
    }
    
    public void SetSwitchLookForValue(boolean v)
    {
        switchLookForValue = v;
    }
    
    @Override
    public boolean EventCheck(double t, double s)
    {
    	boolean rv = false;  // Return value
    	
    	switch(eType)
    	{
    	case Switch:        	
    		if(mtr.getSwitch(0) == switchLookForValue)rv = true;
    		else rv = false;
    		break;
    		
    	case Hardstop:
    		if(Math.abs(cntlr.GetError()) > errorVal)rv = true;
    		else rv = false;
    		break;
    		
    	case None:
    		rv = false;
    		break;
    		
    	default:
    		System.out.printf("<NewJog.eventCheck> Unknown event type");
    		System.exit(1);
    	}
    	return rv;
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
