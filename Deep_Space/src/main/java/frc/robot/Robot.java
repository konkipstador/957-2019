/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
public class Robot extends TimedRobot {

    Drivetrain m_drivetrain = Drivetrain.getInstance();
    Elevator m_elevator = Elevator.getInstance();
    Pathweaver m_pathweaver = Pathweaver.getInstance();
    RobotState m_robotState = RobotState.getInstance();
    Vision m_vision = Vision.getInstance();
    EsCargo m_cargoSystem = EsCargo.getInstance();
    Joystick m_joystick = new Joystick(0);


    boolean m_autoState = false;

    Path[] m_autoPath = new Path[]{Path.ROCKET_RIGHT_1, Path.ROCKET_RIGHT_2, Path.ROCKET_RIGHT_3};

    int m_autoPhase = 0;
    
    public void robotInit() {      
        //m_pathweaver.loadPath(m_autoPath);
        m_robotState.setState(State.GRAB_HATCH);       
        
    }

    public void autonomousInit() {
        //m_drivetrain.resetEncoders();
        //m_drivetrain.resetNavX();
        m_autoPhase = 0;
    }

    

    public void autonomousPeriodic() {

        m_elevator.drive(m_joystick.getRawAxis(1));
        /**Scrimmage Auto Rocket

        switch(m_autoPhase){

            case 0:
                if(m_drivetrain.driveTo(60,0)){
                    m_autoPhase++;
                    m_drivetrain.resetEncoders();
                }
                break;
            case 1:
                if(m_drivetrain.turnTo(61.7)){
                    m_autoPhase++;
                    m_drivetrain.resetEncoders();
                }
                break;
            case 2:
                if(m_drivetrain.driveTo(83,61.7)){
                    m_autoPhase++;
                    m_drivetrain.resetEncoders();
                }
                break;
            case 3:
                if(m_vision.getTargetLocation() == -1000){
                    m_drivetrain.driveStraight(28.6,0.2);
                }else{
                    m_drivetrain.target();
                }
                if(m_drivetrain.getRPM() < 5){
                    m_drivetrain.tank(0,0);
                    m_autoPhase++;
                    m_drivetrain.resetEncoders();
                }
                break;
            case 4:
                if(m_drivetrain.driveTo(-83,61.7)){
                    m_autoPhase++;
                    m_drivetrain.resetEncoders();
                }
                break;
            case 5:
                if(m_drivetrain.turnTo(168.6)){
                    m_autoPhase++;
                    m_drivetrain.resetEncoders();
                }
                break;

            case 6:

                break;
        }
        */

    }

    public void teleopInit() {
    }

    int pressState = 0;
    public void teleopPeriodic() {

        //m_drivetrain.arcadeDrive(-m_joystick.getRawAxis(1), m_joystick.getRawAxis(2));

        // Switch statement controlling what object the robot is grabbing
        switch(pressState){
            case 0:

                m_robotState.setState(State.PLACE_PANEL);
                if(m_joystick.getRawButton(3) && m_elevator.getRaw() < 10)
                    pressState = 1;
                break;

            case 1:

                if(!m_joystick.getRawButton(3))
                    pressState = 2;
                break;

            case 2:

                m_robotState.setState(State.GRAB_CARGO);
                if(m_joystick.getRawButton(3) && m_robotState.state() == State.GRAB_CARGO)
                    pressState = 3;
                break;

            case 3:

                if(!m_joystick.getRawButton(3))
                    pressState = 0;
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
    }

    public enum Buttons{
        ELEVATOR_LOW(2), ELEVATOR_MID(7), ELEVATOR_HIGH(8), ELEVATOR_CARGO(4), AUTOMATED_ACTION(1), 
        TARGET_SWAP(3), MANUAL_RETRACT(6), MANUAL_GRAB(5), MANUAL_PASS(11), SHOOT(12), CLIMB(8);

        // 4 Positions on Cargo Placement
        // Autonomus Enable/Disable button
        // Manual Functions: Retract, Grab, Run Passthrough, Shoot (Nav Controller?)
        // Climbing

        private final int m_button;
	
		// Enum structure constructor
		private Buttons(int button) { 
			m_button = button;
		} 
	
		// Get the m_elevator level that is target
		public int button() 
		{ 
			return m_button;
		} 
    }
}
