/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Elevator.LiftLevels;
import frc.robot.Pathweaver.Path;
import frc.robot.RobotState.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

    Drivetrain m_drivetrain = Drivetrain.getInstance();
    Elevator m_elevator = Elevator.getInstance();
    Pathweaver m_pathweaver = Pathweaver.getInstance();
    RobotState m_robotState = RobotState.getInstance();
    Vision m_vision = Vision.getInstance();
    EsCargo m_cargoSystem = EsCargo.getInstance();
    Joystick m_joystick = new Joystick(0);

    //VL53L0X m_distance = new VL53L0X(6);

    boolean m_autoState = false;

    Path[] m_autoPath = new Path[]{Path.FCS1, Path.FCS2, Path.FCS3};

    int m_autoPhase = 0;
    
    public void robotInit() {      
        m_pathweaver.loadPath(m_autoPath);
        m_robotState.setState(State.PLACE_PANEL);       
        
    }

    public void autonomousInit() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetNavX();
        m_autoPhase = 0;
    }

    

    public void autonomousPeriodic() {
        //m_pathweaver.runPath(0);
        m_cargoSystem.drive(m_joystick.getRawAxis(1));
    }

    public void teleopInit() {
    }

    int pressState = 0;
    public void teleopPeriodic() {

        if(m_joystick.getRawButton(4)) 
            m_drivetrain.target();
        
        
        m_drivetrain.arcadeDrive(m_joystick.getRawAxis(1), m_joystick.getRawAxis(2));

        // Switch statement controlling what object the robot is grabbing
        switch(pressState){
            case 0:
                pressState = 0;
                if(m_joystick.getRawButton(3) && m_elevator.getRaw() < 10)
                    pressState = 1;
                break;

            case 1:
                
                pressState = 1;

                if(!m_joystick.getRawButton(3)){
                    pressState = 2;
                    m_robotState.setState(State.GRAB_CARGO);
                }
                break;

            case 2:

                pressState = 2;
                
                if(m_joystick.getRawButton(3))
                    pressState = 3;
                break;

            case 3:

                pressState = 3;
                
                if(!m_joystick.getRawButton(3)){
                    pressState = 0;
                    m_robotState.setState(State.PLACE_PANEL);
                }
                break;
        }
        
        if(m_robotState.state() == State.PLACE_PANEL){

            if(m_joystick.getRawButton(7))
                m_elevator.setLevel(LiftLevels.HATCH_LOW);

            if(m_joystick.getRawButton(9))
                m_elevator.setLevel(LiftLevels.HATCH_MEDIUM);

            if(m_joystick.getRawButton(11))
                m_elevator.setLevel(LiftLevels.HATCH_HIGH);

            if(m_joystick.getPOV(0) == 0){
                m_elevator.granular(1);
            }

            if(m_joystick.getPOV(0) == 180){
                m_elevator.granular(-1);
            }

            if(m_joystick.getRawButton(1)){
                m_elevator.place();
            }

            if(m_joystick.getRawButton(2)){
                m_elevator.grab();
            }

            if(m_joystick.getRawButton(12)){
                m_cargoSystem.placeCargo();
            }
        }

        if(m_robotState.state() == State.PLACE_CARGO){

            if(m_joystick.getRawButton(1))
                m_cargoSystem.placeCargo();      

            if(m_joystick.getRawButton(7))
                m_elevator.setLevel(LiftLevels.PORT_LOW);

            if(m_joystick.getRawButton(8))
                m_elevator.setLevel(LiftLevels.PORT_CARGO_SHIP);

            if(m_joystick.getRawButton(9))
                m_elevator.setLevel(LiftLevels.PORT_MEDIUM);

            if(m_joystick.getRawButton(11))
                m_elevator.setLevel(LiftLevels.PORT_HIGH);

            if(m_joystick.getPOV(0) == 0){
                m_elevator.granular(1);
            }

            if(m_joystick.getPOV(0) == 180){
                m_elevator.granular(-1);
            }
        }

        m_cargoSystem.run();
        m_elevator.run();
    }

    public void disabledInit(){
        m_drivetrain.tank(0,0);
        m_elevator.reset();
    }

    

    /** Instrum Code */
    public void robotPeriodic(){
        SmartDashboard.putNumber("Gyro", m_drivetrain.getAngle());
        SmartDashboard.putNumber("Elevator Position", m_elevator.getRaw());
        SmartDashboard.putNumber("Arm Position", m_cargoSystem.getArmPosition());  

        SmartDashboard.putBoolean("b1", m_cargoSystem.get1());
        SmartDashboard.putBoolean("b2", m_cargoSystem.get2());
        SmartDashboard.putBoolean("b3", m_cargoSystem.get3()); 
    }
}
