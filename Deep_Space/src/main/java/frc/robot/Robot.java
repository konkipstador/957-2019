/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Pathweaver.Path;
import frc.robot.hatch_mechanisms.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    Drivetrain m_drivetrain = Drivetrain.getInstance();
    Pathweaver m_pathweaver = Pathweaver.getInstance();
    HatchFramework m_hatchGrabber = new Claw();

    Path[] m_autoPath = new Path[]{Path.ROCKET_RIGHT_1, Path.ROCKET_RIGHT_2, Path.ROCKET_RIGHT_3};
    
    public void robotInit() {      
        m_pathweaver.loadPath(m_autoPath);
    }

    public void autonomousInit() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetNavX();
    }

    public void autonomousPeriodic() {
        m_pathweaver.runPath(0);
    }

    public void teleopInit() {
    }

    public void teleopPeriodic() {
    }

    public void testInit() {
    }

    public void testPeriodic() {
    }

}
