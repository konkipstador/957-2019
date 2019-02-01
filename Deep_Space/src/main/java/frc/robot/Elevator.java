package frc.robot;

import frc.robot.RobotState.State;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

	RobotState m_robotState = RobotState.getInstance();

	TalonSRX elevator = new TalonSRX(6);
	int m_targetPosition = 0;
	LiftLevels m_targetLevel = LiftLevels.GROUND;

	double kp = 1;
	double ki = 0;
	double kd = 0;
	double kf = 0;
	int k_cruise = 0;
	int k_accel = 0;
	
	private static Elevator m_elevatorSystem;	// Synchronized Elevator object

	/** Elevator constructor. Don't call directly. Sets up the Elevator Talon Motion Magic settings. */
	public Elevator() {

		// Current Limits
		elevator.configPeakCurrentLimit(29, 20);
		elevator.configPeakCurrentDuration(10, 20);
		elevator.configContinuousCurrentLimit(25, 20);
		elevator.enableCurrentLimit(true);

		// Encoder Setup
		elevator.setSelectedSensorPosition(0, 0, 20);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
        elevator.setSensorPhase(true);
		
		// Configure Motion Magic Constants
		elevator.configMotionCruiseVelocity(k_cruise,20);
		elevator.configMotionAcceleration(k_accel, 20);    
 		elevator.config_kP(0, kp, 20);
		elevator.config_kI(0, ki, 20);
        elevator.config_kD(0, kd, 20);
		elevator.config_kF(0, kf, 20);
	}
	
	/** Sets the elevator position.
	 * @param level: LiftLevels value storing elevator level information.
	 */
	public void setLevel(LiftLevels level) {
		m_targetLevel = level;
		m_targetPosition = level.encoderPosition();
		elevator.set(ControlMode.MotionMagic, m_targetPosition);
	}

	/** @return LiftLevels object storing elevator level information. */
	public LiftLevels getLevel(){
		return m_targetLevel;
	}

	/** @return Raw encoder position of the elevator. */
	public int getRaw() {
		return 0;
	}

	/** @return Maximum allowed drivetrain speed. Varies based on height. */
	public double maximumDriveSpeed() {
		return 1;
	}

	/** @return Returns a synchronized elevator object. */
	public static synchronized Elevator getInstance(){
        if (m_elevatorSystem == null)
			m_elevatorSystem = new Elevator();

        return m_elevatorSystem;     
	}
	
	/** Enum storing elevator positional information and maximum allowed drive speed. */
	public enum LiftLevels{
    
		// Levels of the hatch ports
		HATCH_LOW(0), HATCH_MEDIUM(0), HATCH_HIGH(0),
		// Levels of the cargo ports
		PORT_LOW(0), PORT_CARGO_SHIP(0), PORT_MEDIUM(0), PORT_HIGH(0),
		// Other Levels
		GROUND(0);
		
		// Placeholder variables for the Enumerator structure
		private final int m_encoderPosition;
	
		// Enum structure constructor
		private LiftLevels(int encoderPosition) { 
			m_encoderPosition = encoderPosition;
		} 
	
		// Get the elevator level that is target
		public int encoderPosition() 
		{ 
			return m_encoderPosition;
		} 
	}
}