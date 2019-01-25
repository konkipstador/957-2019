package frc.robot.hatch_mechanisms;

import edu.wpi.first.wpilibj.Solenoid;

/** Controls a hatch grabbing and placing mechanism created by Joe and his team.
 * 
 *  Uses a claw to grab and shoot panels and extends out the mechanism to place on
 *  the Rocket. Also has an ultrasonic sensor to check if a panel has been grabbed. */

public class Claw implements HatchFramework {

    Solenoid m_grabberPiston = new Solenoid(piston1Channel);

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
        m_grabberPiston.set(false);
    }

    /** Releases the hatch */
    public void releaseHatch(){
        m_grabberPiston.set(true);
    }

    /** Returns if the grabber mechanism is extended */
    public boolean extendState(){
        return extenderPiston.get();
    }

    /** Returns if the grabber mechanism is grabbing */
    public boolean grabState(){
        // This value is inverted because when the grabber piston is extended, 
        // the mechanism is releasing the hatch.
        return !m_grabberPiston.get();
    }
}