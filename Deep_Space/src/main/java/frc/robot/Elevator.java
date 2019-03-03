package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

	RobotState m_robotState = RobotState.getInstance();
	double m_targetPosition = 0;
	LiftLevels m_targetLevel = LiftLevels.HATCH_LOW;
	
	CANSparkMax m_spark = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANEncoder m_encoder = m_spark.getEncoder();
	CANPIDController m_pidController = m_spark.getPIDController();

	DoubleSolenoid m_grabber = new DoubleSolenoid(6,7);

	double kP = 0.00008;
    double kI = 5e-6;
    double kD = 0.00001;
    double kIz = 2;
    double kFF = 0.0002;
    int maxRPM = 4500;
    int maxVel = 4500;
	double maxAcc = 3500;
	
	double grabHeight = 0;

	double m_position = 0;
	boolean m_placing = false;
	
	private static Elevator m_elevatorSystem;	// Synchronized m_Elevator object

	/** Elevator constructor. */
	private Elevator() {

		m_spark.setInverted(false);
		m_spark.setIdleMode(IdleMode.kBrake);
		m_pidController.setP(kP);
		m_pidController.setI(kI);
		m_pidController.setD(kD);
		m_pidController.setIZone(kIz);
		m_pidController.setFF(kFF);
		m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
		m_pidController.setSmartMotionMinOutputVelocity(0, 0);
		m_pidController.setSmartMotionMaxAccel(maxAcc,0);
		m_pidController.setOutputRange(-1, 1);		
	}

	public void drive(double input){
		m_spark.set(input);
	}

	/** Function used to test the elevator with a joystick. */
	public void granular(double input){
		m_targetPosition = m_targetPosition + input/6;
		if(m_targetPosition < 0.6){
			m_targetPosition = 0.6;
		}
		if(m_targetPosition > 82){
			m_targetPosition = 82;
		}
		SmartDashboard.putNumber("pos", m_encoder.getPosition());
		SmartDashboard.putNumber("tg", m_position);
	}

	/** Sets the m_elevator position.
	 * @param level: LiftLevels value storing m_elevator level information.
	 */
	public void setLevel(LiftLevels level) {
		m_targetLevel = level;
		m_targetPosition = level.encoderPosition();
		m_pidController.setReference(level.encoderPosition(), ControlType.kSmartMotion);
	}


	public void run(){

		double finalPosition = 0;

		if(m_targetPosition < 0.6){
			m_targetPosition = 0.6;
		}
		if(m_targetPosition > 81){
			m_targetPosition = 81;
		}

		if(m_placing){
			finalPosition = m_targetPosition + 7;
		}else{
			finalPosition = m_targetPosition;
		}

		if(finalPosition < 0.6){
			finalPosition = 0.6;
		}
		if(finalPosition > 81){
			finalPosition = 81;
		}
		m_pidController.setReference(finalPosition, ControlType.kSmartMotion);
	}

	public void extend(){
		m_grabber.set(Value.kForward);
	}

	public void retract(){
		m_grabber.set(Value.kReverse);
	}

	public void place(){
		m_placing = false;
	}

	public void grab(){
		grabHeight = m_encoder.getPosition();
		m_placing = true;
	}

	public void reset(){
		m_spark.set(0);
	}

	/** @return LiftLevels object storing m_elevator level information. */
	public LiftLevels getLevel(){
		return m_targetLevel;
	}

	/** @return Raw encoder position of the m_elevator. */
	public double getRaw() {
		return m_encoder.getPosition();
	}

	/** @return Maximum allowed drivetrain speed. Varies based on height. */
	public double maximumDriveSpeed() {
		if(m_encoder.getPosition() > 50){
			return 0.2;
		}

		if(m_encoder.getPosition() > 20){
			return 0.5;
		}

		return 1;
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
		HATCH_LOW(0.6), HATCH_MEDIUM(40), HATCH_HIGH(76),
		// Levels of the cargo ports
		PORT_LOW(0.6), PORT_CARGO_SHIP(40), PORT_MEDIUM(40), PORT_HIGH(82),
		// Other Levels
		GROUND(0.6);
		
		// Placeholder variables for the Enumerator structure
		private final double m_encoderPosition;
	
		// Enum structure constructor
		private LiftLevels(double encoderPosition) { 
			m_encoderPosition = encoderPosition;
		} 
	
		// Get the m_elevator level that is target
		public double encoderPosition() 
		{ 
			return m_encoderPosition;
		} 
	}
}