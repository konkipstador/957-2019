package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

	RobotState m_robotState = RobotState.getInstance();

	TalonSRX m_elevator = new TalonSRX(5);
	int m_targetPosition = 0;
	LiftLevels m_targetLevel = LiftLevels.GROUND;

	// PIDF CONSTANTS
	double kp = 0.8;
	double ki = 0.0005;
	double kd = 1.5;
	double kf = 0.372;
	int k_cruise = 2000;
	int k_accel = 2000;
	
	private static Elevator m_elevatorSystem;	// Synchronized m_Elevator object

	/** m_Elevator constructor. Don't call directly. Sets up the m_Elevator Talon Motion Magic settings. */
	public Elevator() {

		// Current Limits
		m_elevator.configPeakCurrentLimit(29, 20);
		m_elevator.configPeakCurrentDuration(10, 20);
		m_elevator.configContinuousCurrentLimit(25, 20);
		m_elevator.enableCurrentLimit(true);

		// Encoder Setup
		m_elevator.setSelectedSensorPosition(0, 0, 20);
		m_elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
		m_elevator.setSensorPhase(true);
		m_elevator.setInverted(false);
		
		// Configure Motion Magic Constants
		m_elevator.configMotionCruiseVelocity(k_cruise,20);
		m_elevator.configMotionAcceleration(k_accel, 20);    
 		m_elevator.config_kP(0, kp, 20);
		m_elevator.config_kI(0, ki, 20);
        m_elevator.config_kD(0, kd, 20);
		m_elevator.config_kF(0, kf, 20);
		m_elevator.config_IntegralZone(0, 1500);
	}

	/** Sets the m_elevator position.
	 * @param level: LiftLevels value storing m_elevator level information.
	 */
	public void setLevel(LiftLevels level) {
		m_targetLevel = level;
		m_targetPosition = level.encoderPosition();
		m_elevator.set(ControlMode.MotionMagic, m_targetPosition);

		SmartDashboard.putNumber("pos", m_elevator.getSelectedSensorPosition());
	}

	public void reset(){
		m_elevator.set(ControlMode.PercentOutput, 0);
	}

	/** @return LiftLevels object storing m_elevator level information. */
	public LiftLevels getLevel(){
		return m_targetLevel;
	}

	/** @return Raw encoder position of the m_elevator. */
	public int getRaw() {
		return 0;
	}

	/** @return Maximum allowed drivetrain speed. Varies based on height. */
	public double maximumDriveSpeed() {
		if(m_targetLevel.encoderPosition() > 18000){
			return 2600;
		}

		if(m_targetLevel.encoderPosition() > 12000){
			return 4200;
		}

		return 5800;
	}

	/** @return Returns a synchronized m_elevator object. */
	public static synchronized Elevator getInstance(){
        if (m_elevatorSystem == null)
			m_elevatorSystem = new Elevator();

        return m_elevatorSystem;     
	}
	
	/** Enum storing m_elevator positional information and maximum allowed drive speed. */
	public enum LiftLevels{
    
		// Levels of the hatch ports
		HATCH_LOW(200), HATCH_MEDIUM(15000), HATCH_HIGH(0),
		// Levels of the cargo ports
		PORT_LOW(0), PORT_CARGO_SHIP(0), PORT_MEDIUM(0), PORT_HIGH(21925),
		// Other Levels
		GROUND(0);
		
		// Placeholder variables for the Enumerator structure
		private final int m_encoderPosition;
	
		// Enum structure constructor
		private LiftLevels(int encoderPosition) { 
			m_encoderPosition = encoderPosition;
		} 
	
		// Get the m_elevator level that is target
		public int encoderPosition() 
		{ 
			return m_encoderPosition;
		} 
	}
}