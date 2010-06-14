package TranRunJLiteSamples;

import java.util.ArrayList;

import TranRunJLite.*;

public class Homing extends TrjTask 
{
	protected double dtNominal;
	protected ArrayList<NewJog> joggerList;
	protected ArrayList<Motor> motorList;
	protected ArrayList<MotorCosineProfileP> profileList;
	protected boolean homingDone = false;
	protected double [] preHomeLocations = null;
    boolean [] stopOnEvent = null;
    boolean [] switchLookForVal = null;
    EventType [] event = null;
    double [] jogVelHome = null;
    double [] errorVal = null;
    double [] initialLocation = new double[6];

	// States:
	public static final int off = 0;
	public static final int preHoming  = 1;
	public static final int waitForProfileOff = 2;
	public static final int homing0 = 3;
	public static final int homing1 = 4;
	public static final int homing2 = 5;
	public static final int cleanUpHoming = 6;
	public static final int backToPark = 7;
	public static final int waitForProfileOff2 = 8;
	public static final int idle = 9;
	
	// Commands
	public static final int startHoming = 0;
	
	public boolean GetHomingDone()
	{
		return homingDone;
	}
	
    Homing(String name, TrjSys sys, int initialState,
            boolean taskActive, double dtNominal, 
            ArrayList<NewJog> joggerList, ArrayList<Motor> motorList,
            ArrayList<MotorCosineProfileP> mcp, double [] preHomeLocations,
            boolean [] stopOnEvent, boolean [] switchLookForVal, 
            EventType [] event, double [] jogVelHome,
            double [] errorVal
            )
    {
        super(name, sys, initialState, taskActive);
        this.dtNominal = dtNominal;
        this.joggerList = joggerList;
        this.motorList = motorList;
        this.preHomeLocations = preHomeLocations;
        this.profileList = mcp;
        this.stopOnEvent = stopOnEvent;
        this.switchLookForVal = switchLookForVal;
        this.event = event;
        this.jogVelHome = jogVelHome;
        this.errorVal = errorVal;
        
        stateNames.add("off");
        stateNames.add("preHoming");
        stateNames.add("waitForProfileOff");
        stateNames.add("homing0");
        stateNames.add("homing1");
        stateNames.add("homing2");
        stateNames.add("cleanUpHoming");
        stateNames.add("backToPark");
        stateNames.add("waitForProfileOff2");
        stateNames.add("idle");
    }

    public boolean RunTaskNow(TrjSys sys)
    {
        return CheckTime(sys.GetRunningTime());
    }

    public boolean RunTask(TrjSys sys)
    {
        int cmd;
        nextState = -1;
        double t = sys.GetRunningTime();
        
        switch(currentState)
        {
        	case off:  // Just wait for a command
        		cmd = GetCommand();
        		if(cmd == -1)break;
        		else if(cmd == startHoming)nextState = preHoming;
        		break;
        		
        	case preHoming:
        		// Use the profiler to move to a pre-homing position
        		if(runEntry)
        		{
        			// Record the present location (Parked position)
        			for(int j = 0; j < 6; j++)
        			{
        				initialLocation[j] = motorList.get(j).getEngrgPos();
        			}
        			
        			int i = 0;
        			for(MotorCosineProfileP mp : profileList)
        			{
        				// Turn profilers on
        				mp.setNewProfile(preHomeLocations[i] + initialLocation[i]);
        				mp.SetCommand(ProfileGenerator.START_PROFILE);
        				mp.ClearProfileDoneFlag();  //To avoid a race
        				mp.ActivateTask();  //Make sure it's activated
        				i++;
        			}
        		}
        		
        		// Wait for all profiles to complete before exiting
        		boolean allDone = true;  // Set to false if any profiles not done
    			for(MotorCosineProfileP mp : profileList)
    			{
    				if(!mp.GetDoneFlag())allDone = false;
    			}
    			if(allDone)
    				{
    					// Turn off the profile tasks
        				for(MotorCosineProfileP mp : profileList)
        				{
        					mp.SetCommand(ProfileGenerator.TURN_PROFILE_OFF);
        				}    				
        				nextState = waitForProfileOff;
    				}
    			break;
        		
        	case waitForProfileOff:
        		boolean allOff = true;
    			for(MotorCosineProfileP mp : profileList)
    			{
    				if(mp.GetState() != ProfileGenerator.PROFILE_OFF)allOff = false;
    			}
    			if(allOff)nextState = homing0;
    			break;
        		
        	case homing0:
    			int []axesToHome = {0,1, 3, 5};
    			int n = axesToHome.length;
    			
        		if(runEntry)
        		{
        			homingDone = false;
        			// Set up all of the jog tasks, ready for homing
        			for(int j = 0; j <n; j++)
        			{
        				int i = axesToHome[j];
        				NewJog jj = joggerList.get(i);  // Select axes to home
        	            jj.SetEventParameters(stopOnEvent[i], switchLookForVal[i],
        	            		errorVal[i], event[i], jogVelHome[i]);
        	            jj.SetCommand(NewJog.START_JOGGING);
        	            jj.ActivateTask(); // Make sure task is active 
        			}
        			
        			// Axes 0, 1, 4 and 5 (waist, shoulder, wrist-rotate, gripper) can
        			// be homed simultaneously. The others must be done one at a time,
        			// in order.
        		}
        		// Wait for all axes in this group to find home
        		boolean done = true;
    			for(int j = 0; j <n; j++)
    			{
    				int i = axesToHome[j];
    				NewJog jj = joggerList.get(i);  // Select axis
    				if(!jj.GetEventFlag())done = false;
    			}
    			
    			if(done)nextState = homing1; // Move on to remaining axes, one at a time    			
        		break;
        		
        	case homing1: // Home axis 2
				int i = 2;
				NewJog jj = joggerList.get(i);  // Select axes to home
        		if(runEntry)
        		{
    	            jj.SetEventParameters(stopOnEvent[i], switchLookForVal[i],
    	            		errorVal[i], event[i], jogVelHome[i]);
    	            jj.SetCommand(NewJog.START_JOGGING);
    	            jj.ActivateTask(); // Make sure task is active 
        		}
        		
        		// Check to see if this axis is homed
        		if(jj.GetEventFlag())nextState = homing2;
        		break;
        		
        	case homing2: // Home axis 4
				i = 4;
				jj = joggerList.get(i);  // Select axes to home
        		if(runEntry)
        		{

        			// Set up all of the jog tasks, ready for homing
    	            jj.SetEventParameters(stopOnEvent[i], switchLookForVal[i],
    	            		errorVal[i], event[i], jogVelHome[i]);
    	            jj.SetCommand(NewJog.START_JOGGING);
    	            jj.ActivateTask(); // Make sure task is active 
        		}
        		
        		// Check to see if this axis is homed
        		if(jj.GetEventFlag())nextState = cleanUpHoming;
        		break;
        		
        	case cleanUpHoming:  // Set homing information and turn joggers off
        	{
        		if(runEntry)
        		{
        			for(NewJog jg : joggerList)
        				jg.SetCommand(NewJog.STOP_JOGGING);
        			for(int k = 0; k < 6; k++)
        			{
        				// copy home information to motors
        				jj = joggerList.get(k);
        				double h = jj.GetEventLocation();
        				motorList.get(k).SetHomePosition(h);
        			}
        		}
        		// Check to see that all jogger tasks are in the OFF state
        		boolean joggersOff = true;
        		for(NewJog jg : joggerList)
        			if(jg.GetState() != Jogger2.OFF)joggersOff = false;
        		if(joggersOff)nextState = backToPark;
        		break;
        	}
        	
        	case backToPark:  // Return to robot to its initial position
        	{
        		if(runEntry)
        		{
        			i = 0;
        			for(MotorCosineProfileP mp : profileList)
        			{
        				// Turn profilers on
        				mp.setNewProfile(initialLocation[i]);
        				mp.SetCommand(ProfileGenerator.START_PROFILE);
        				mp.ClearProfileDoneFlag();  //To avoid a race
        				mp.ActivateTask();  //Make sure it's activated
        				i++;
        			}
        		}
        		// Wait for all profiles to complete before exiting
        		allDone = true;  // Set to false if any profiles not done
    			for(MotorCosineProfileP mp : profileList)
    			{
    				if(!mp.GetDoneFlag())allDone = false;
    			}
    			if(allDone)
    				{
    					// Turn off the profile tasks
        				for(MotorCosineProfileP mp : profileList)
        				{
        					mp.SetCommand(ProfileGenerator.TURN_PROFILE_OFF);
        				}    				
        				nextState = waitForProfileOff2;
    				}
    			break;
        	}
        		
        	case waitForProfileOff2:
        		allOff = true;
    			for(MotorCosineProfileP mp : profileList)
    			{
    				if(mp.GetState() != ProfileGenerator.PROFILE_OFF)allOff = false;
    			}
    			if(allOff)
    				{
    				nextState = idle;
    				homingDone = true;
    				}
    			break;
        	
        	case idle:  // Done with homing. Nothing to do in this state
        		break;
        }
        return false;
    }
}
