package frc.robot.hatch_mechanisms;

/** Controls a passive hatch grabbing and placing mechanism created by Joe and his team.
 * 
 *  Only uses the piston and ultrasonic sensor created in HatchFramework.java to extend
 *  the mechanism to extend beyond the frame perimeter and to sense the distance of
 *  hatches respectively.*/
public class PassiveHatch implements HatchFramework {

    /** Extends the hatch grabber mechanism. */
    public void extend(){
        extenderPiston.set(true);
    }

    /** Retracts the hatch grabber mechanism. */
    public void retract(){
        extenderPiston.set(false);
    }

    /** Returns the distance to the hatch or wall in inches. */
    public void distanceToHatch(){
        rangeFinder.getRangeInches();
    }

    /** Unused in passive mechanism */
    public void grabHatch(){}

    /** Unused in passive mechanism */
    public void releaseHatch(){}   
}