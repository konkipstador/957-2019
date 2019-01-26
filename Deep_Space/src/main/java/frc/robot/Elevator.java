package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

	private static Elevator m_elevatorSystem;	// Synchronized Elevator object

	TalonSRX elevator = new TalonSRX(6);	// Elevator Talon
	int targetPosition = 0;
	int currentPos = 0;	
	boolean getCurrentPosition = true;
		
	public Elevator() {

		elevator.configPeakCurrentLimit(29, 20);
		elevator.configPeakCurrentDuration(10, 20);
		elevator.configContinuousCurrentLimit(25, 20);
		elevator.enableCurrentLimit(true);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        elevator.setSensorPhase(true);
        elevator.config_IntegralZone(0, 50, 20);
		elevator.configMotionCruiseVelocity(2000,20);
		elevator.configMotionAcceleration(2000, 20);
        elevator.setSelectedSensorPosition(0, 0, 20);
        
        double kp = 1.5493;
        double ki = 0.016;
        double kd = 15.493;
        double kf = .2177;
		
		elevator.config_kP(0, kp, 20);
		elevator.config_kI(0, ki, 20);
        elevator.config_kD(0, kd, 20);
        elevator.config_kF(0, kf, 20);
	}
	
	public void setLevel(LiftLevels level) {	

		getCurrentPosition = true;
		targetPosition = level.encoderPosition();
		elevator.set(ControlMode.MotionMagic, targetPosition);
	}

	public int getRaw() {
		return elevator.getSelectedSensorPosition(0);
	}

	public double maximumDriveSpeed() {
		return 1;
	}

	public static synchronized Elevator getInstance(){
        if (m_elevatorSystem == null)
			m_elevatorSystem = new Elevator();

        return m_elevatorSystem;     
    }
}