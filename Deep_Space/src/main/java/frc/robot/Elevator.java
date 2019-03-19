package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Elevator {

	RobotState m_robotState = RobotState.getInstance();
	double m_targetPosition = 0;
	LiftLevels m_targetLevel = LiftLevels.HATCH_LOW;
	
	CANSparkMax m_spark = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANEncoder m_encoder = m_spark.getEncoder();
	CANPIDController m_pidController = m_spark.getPIDController();

	DoubleSolenoid m_grabber = new DoubleSolenoid(12, 0,1);
	//DoubleSolenoid m_grabber = new DoubleSolenoid(6, 6,7);

	double kP = 0.00008;
    double kI = 5e-6;
    double kD = 0.00001;
    double kIz = 2;
    double kFF = 0.0002;
    int maxRPM = 5700;
    int maxVel = 5700;
	double maxAcc = 3750;
	
	double grabHeight = 0;

	double m_position = 0;
	boolean m_placing = false;
	boolean m_extended = false;
	
	private static Elevator m_elevatorSystem;	// Synchronized m_Elevator object

	/** Elevator constructor. */
	private Elevator() {

		m_spark.setInverted(false);
		m_spark.setIdleMode(IdleMode.kBrake);
		m_spark.setSmartCurrentLimit(30);
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
		m_targetPosition = m_targetPosition + input/3;
		if(m_targetPosition < 0.6){
			m_targetPosition = 0.6;
		}
		if(m_targetPosition > 160){
			m_targetPosition = 160;
		}
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
		if(m_targetPosition > 160){
			m_targetPosition = 160;
		}

		if(m_placing){
			finalPosition = m_targetPosition + 16;
		}else{
			finalPosition = m_targetPosition;
		}

		if(finalPosition < 0.6){
			finalPosition = 0.6;
		}
		if(finalPosition > 160){
			finalPosition = 160;
		}
		m_pidController.setReference(finalPosition, ControlType.kSmartMotion);
	}

	public void extend(){
		m_grabber.set(Value.kForward);
		m_extended = true;
	}

	public void retract(){
		m_grabber.set(Value.kReverse);
		m_extended = false;
	}

	public boolean isExtended(){
		return m_extended;
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
		if(m_encoder.getPosition() > 100){
			return 0.2;
		}

		if(m_encoder.getPosition() > 40){
			return 0.5;
		}

		return 1;
	}

	public double getCurrent(){
		return m_spark.getOutputCurrent();
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
		HATCH_LOW(1.2, "Hatch Low"), HATCH_MEDIUM(80, "Hatch Medium"), HATCH_HIGH(142, "Hatch High"),
		// Levels of the cargo ports
		PORT_LOW(1.2, "Cargo Low"), PORT_CARGO_SHIP(50, "Cargo Ship"), PORT_MEDIUM(80, "Cargo Medium"), PORT_HIGH(164, "Cargo High"),
		// Other Levels
		GROUND(1.2, "very low, much ground");
		
		// Placeholder variables for the Enumerator structure
		private final double m_encoderPosition;
		private final String m_identifier;
	
		// Enum structure constructor
		private LiftLevels(double encoderPosition, String identifer) { 
			m_encoderPosition = encoderPosition;
			m_identifier = identifer;
		} 
	
		// Get the m_elevator level that is target
		public double encoderPosition() 
		{ 
			return m_encoderPosition;
		} 

		public String identifier() 
		{ 
			return m_identifier;
		} 
	}
}