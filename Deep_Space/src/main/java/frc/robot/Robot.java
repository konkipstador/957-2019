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
import frc.robot.hatch_mechanisms.Claw;
import frc.robot.hatch_mechanisms.HatchFramework;

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
    HatchFramework m_hatchGrabber = new Claw();
    EsCargo m_cargoSystem = EsCargo.getInstance();
    Joystick m_joystick = new Joystick(0);

    /** Elevator: Low Port/Floor */
    int k_button1 = 1;
    /** Elevator: Medium Port */ 
    int k_button2 = 2;
    /** Elevator: High Port */
    int k_button3 = 2;
    /** Swap between Cargo and Hatch grabbing OR switch target between Rocket and Cargo Ship */
    int k_button4 = 3;
    /** Perform automated action or cancel automated action */
    int k_button5 = 2;
    /** Actuate leapfrog */
    int k_button6 = 1;
    /** Change claw grab state OR manually shoot cargo */
    int k_button7 = 6;
    /** Extend claw beyond bumpers OR run passthrough manually */
    int k_button8 = 7; 

    boolean m_autoState = false;

    Path[] m_autoPath = new Path[]{Path.ROCKET_RIGHT_1, Path.ROCKET_RIGHT_2, Path.ROCKET_RIGHT_3};
    
    public void robotInit() {      
        m_pathweaver.loadPath(m_autoPath);
        m_robotState.setState(State.GRAB_HATCH);
        
    }

    public void autonomousInit() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetNavX();
    }

    public void autonomousPeriodic() {
        //m_pathweaver.runPath(0);
        m_drivetrain.driveToPosition(0.464*36);
    }

    public void teleopInit() {
    }

    public void teleopPeriodic() {
        SmartDashboard.putNumber("Temperatures", m_drivetrain.getTemp()[0]);

        if(m_robotState.state() == State.GRAB_HATCH){

            if(m_autoState){

                // TODO: Autonomus panel grabbing


                if(m_joystick.getRawButtonReleased(Buttons.AUTOMATED_ACTION.button())){
                    m_autoState = false;
                }

            }else{
                if(m_joystick.getRawButton(Buttons.ELEVATOR_LOW.button())){
                    m_elevator.setLevel(LiftLevels.HATCH_LOW);
                }

                if(m_joystick.getRawButtonReleased(Buttons.AUTOMATED_ACTION.button()) && m_elevator.getLevel() == LiftLevels.HATCH_LOW){
                    m_autoState = true;
                }

                if(m_joystick.getRawButtonReleased(Buttons.TARGET_SWAP.button()) && m_elevator.getLevel() == LiftLevels.HATCH_LOW){
                    m_elevator.setLevel(LiftLevels.PORT_LOW);
                    m_robotState.setState(State.GRAB_CARGO);
                }
            }      

        }

        if(m_robotState.state() == State.GRAB_CARGO){

            if(m_joystick.getRawButtonReleased(Buttons.TARGET_SWAP.button())){
                m_elevator.setLevel(LiftLevels.HATCH_LOW);
                m_robotState.setState(State.GRAB_HATCH);
            }
            
        }

        if(m_robotState.state() == State.PASSTHROUGH){
            
        }

        if(m_robotState.state() == State.PLACE_PANEL){

            if(m_autoState){

                // TODO: Autonomus panel placement


                if(m_joystick.getRawButtonReleased(Buttons.AUTOMATED_ACTION.button())){
                    m_autoState = false;
                }

            }else{
                if(m_joystick.getRawButton(Buttons.ELEVATOR_LOW.button())){
                    m_elevator.setLevel(LiftLevels.HATCH_LOW);
                }
    
                if(m_joystick.getRawButton(Buttons.ELEVATOR_MID.button())){
                    m_elevator.setLevel(LiftLevels.HATCH_MEDIUM);
                }
    
                if(m_joystick.getRawButton(Buttons.ELEVATOR_HIGH.button())){
                    m_elevator.setLevel(LiftLevels.HATCH_HIGH);
                }

                if(m_joystick.getRawButtonReleased(Buttons.AUTOMATED_ACTION.button())){
                    m_autoState = true;
                }
            }     
            
            m_cargoSystem.run();
        }

        if(m_robotState.state() == State.PLACE_CARGO){

            if(m_autoState){

                // TODO: Autonomus cargo placement


                if(m_joystick.getRawButtonReleased(Buttons.AUTOMATED_ACTION.button())){
                    m_autoState = false;
                }

            }else{
            
                if(m_joystick.getRawButton(Buttons.ELEVATOR_LOW.button())){
                    m_elevator.setLevel(LiftLevels.PORT_LOW);
                }

                if(m_joystick.getRawButton(Buttons.ELEVATOR_MID.button())){
                    m_elevator.setLevel(LiftLevels.PORT_MEDIUM);
                }

                if(m_joystick.getRawButton(Buttons.ELEVATOR_HIGH.button())){
                    m_elevator.setLevel(LiftLevels.PORT_HIGH);
                }
            }
        }
    }

    public void disabledInit(){
        m_drivetrain.setLeft(0);
        m_drivetrain.setRight(0);
        m_elevator.reset();

        SmartDashboard.putString("data", m_joystick.getName());
    }

    public enum Buttons{
        ELEVATOR_LOW(2), ELEVATOR_MID(7), ELEVATOR_HIGH(8), ELEVATOR_CARGO(4), AUTOMATED_ACTION(1), 
        TARGET_SWAP(3), MANUAL_RETRACT(6), MANUAL_GRAB(5), MANUAL_PASS(11), SHOOT(12), CLIMB(8);

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
