package frc.robot.hatch_mechanisms;

import edu.wpi.first.wpilibj.Solenoid;

/** Controls a hatch grabbing and placing mechanism created by Casey and his team.
 * 
 *  Uses 3 solenoids to control the grabber mechanism. Two grab the panel itself and
 *  one extends to place the panel. Also has an ultrasonic sensor to check if a panel
 *  has been grabbed. */

public class OtisIntake implements HatchFramework {

    Solenoid m_grabberPiston1 = new Solenoid(piston1Channel);
    Solenoid m_grabberPiston2 = new Solenoid(piston2Channel);

    /** Extends the hatch grabber mechanism. */
    public void extend(){
        extenderPiston.set(true);    
    }

    /** Retracts the hatch grabber mechanism. */
    public void retract(){
        extenderPiston.set(false);
    }

    /** Returns the distance to the hatch or wall in inches. */
    public double distanceToHatch(){
        return rangeFinder.getRangeInches();
    }

    /** Grabs the hatch */
    public void grabHatch(){
        m_grabberPiston1.set(false);
        m_grabberPiston2.set(false);
    }

    /** Releases the hatch */
    public void releaseHatch(){
        m_grabberPiston1.set(true);
        m_grabberPiston2.set(true);
    } 
    
    /** Returns if the grabber mechanism is extended */
    public boolean extendState(){
        return extenderPiston.get();
    }

    /** Returns if the grabber mechanism is grabbing */
    public boolean grabState(){
        // This value is inverted because when the grabber piston is extended, 
        // the mechanism is releasing the hatch.
        return !m_grabberPiston1.get();
    }
}