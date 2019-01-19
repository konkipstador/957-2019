package frc.robot.hatch_mechanisms;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

public interface HatchFramework{

    /** Piston used to extend all mechanisms outside the frame perimeter. */
    Solenoid extenderPiston = new Solenoid(0);

    /** Find the distance to the hatch panel or wall. Used in Auto to see if the hatch panel
     *  was grabbed. */
    Ultrasonic rangeFinder = new Ultrasonic(0, 1);

    /** PCM channel for the first piston of a mechanism. Use when defining the first piston */
    int piston1Channel = 1;

    /** PCM channel for the second piston of a mechanism. Use when defining the first piston */
    int piston2Channel = 2;

    /** Skeleton function defined in individual mechanism classes */
    public void extend();
    /** Skeleton function defined in individual mechanism classes */
    public void retract();
    /** Skeleton function defined in individual mechanism classes */
    public void grabHatch();
    /** Skeleton function defined in individual mechanism classes */
    public void releaseHatch();
    /** Skeleton function defined in individual mechanism classes */
    public void distanceToHatch();
}