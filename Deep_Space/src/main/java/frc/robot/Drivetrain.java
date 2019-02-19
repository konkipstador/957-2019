package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import frc.libraries.MiniPID;

/**
 * Class to operate the drivetrain of the robot.
 * 
 * Automatically creates 4 Spark Max objects and master/slaves them together.
 * Has built in arcade drive features and direct commands to set drivetrain
 * speeds, useful for Auto.
 */
public class Drivetrain {

    AHRS m_navx = new AHRS(Port.kMXP);
    Vision m_vision = Vision.getInstance();
    Elevator m_elevator = Elevator.getInstance();

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    double v_vkp = 0.02;
    double v_vki = 0.;
    double v_vkd = 0;
    MiniPID c_visionLoop = new MiniPID(v_vkp,v_vki,v_vki);

    TalonSRX m_rightDM = new TalonSRX(11);
    CANSparkMax m_rightNeoMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_rightNeoSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_rightEncoder = m_rightNeoMaster.getEncoder();
    
    TalonSRX m_leftDM = new TalonSRX(10);
    CANSparkMax m_leftDS1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_leftDS2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_leftEncoder = m_leftDS1.getEncoder();

    private static Drivetrain m_drivetrain = null;

    private static final int k_freeCurrentLimit = 40;
    private static final int k_stallCurrentLimit = 40;

    double m_rightEncoderOffset = 0;
    double m_leftEncoderOffset = 0;

    FeedbackDevice talon = FeedbackDevice.SoftwareEmulatedSensor;

    double m_magicNumber = 0;

    /**
     * Drivetrain constructor, which is called automatically when an instance of the
     * drivetrain is asked for. You should not call directly.
     */
    public Drivetrain(){
        m_rightNeoMaster.follow(ExternalFollower.kFollowerPhoenix, 11);
        m_rightNeoSlave.follow(ExternalFollower.kFollowerPhoenix, 11);

        m_leftDS1.follow(ExternalFollower.kFollowerPhoenix, 10);
        m_leftDS2.follow(ExternalFollower.kFollowerPhoenix, 10);
        
        m_rightNeoMaster.setIdleMode(IdleMode.kCoast);
        m_rightNeoSlave.setIdleMode(IdleMode.kCoast);
        m_leftDS1.setIdleMode(IdleMode.kCoast);
        m_leftDS2.setIdleMode(IdleMode.kCoast);
        //m_leftNeoS.setInverted(true);

        // Neo Current Limits
        m_rightNeoMaster.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_rightNeoSlave.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_leftDS1.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_leftDS2.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);

        /** 
		m_rightVelocity.setP(kP);
		m_rightVelocity.setI(kI);
		m_rightVelocity.setD(kD);
		m_rightVelocity.setIZone(kIz);
		m_rightVelocity.setFF(kFF);
		m_rightVelocity.setSmartMotionMaxVelocity(maxVel, 0);
		m_rightVelocity.setSmartMotionMinOutputVelocity(0, 0);
		m_rightVelocity.setSmartMotionMaxAccel(maxAcc,0);
        m_rightVelocity.setOutputRange(-1, 1);	
        m_rightVelocity.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        m_rightNeoM.setClosedLoopRampRate(0.5);
        m_rightVelocity.setSmartMotionAllowedClosedLoopError(1, 0);
        	
        m_leftVelocity.setP(kP);
		m_leftVelocity.setI(kI);
		m_leftVelocity.setD(kD);
		m_leftVelocity.setIZone(kIz);
		m_leftVelocity.setFF(kFF);
		m_leftVelocity.setSmartMotionMaxVelocity(maxVel, 0);
		m_leftVelocity.setSmartMotionMinOutputVelocity(0, 0);
		m_leftVelocity.setSmartMotionMaxAccel(maxAcc,0);
        m_leftVelocity.setOutputRange(-1, 1);
        m_leftVelocity.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        m_leftNeoM.setClosedLoopRampRate(.5);
        m_leftVelocity.setSmartMotionAllowedClosedLoopError(1, 0);
         */

        double kp = 0;
        double ki = 0;
        double kd = 0;
        double kf = .00000000000000000001;
        int cruise = 0;
        int accel = 0;

        m_leftDM.config_kP(0, kp);
        m_leftDM.config_kI(0, ki);
        m_leftDM.config_kD(0, kd);
        m_leftDM.config_kF(0, kf);
        m_leftDM.configMotionCruiseVelocity(0, cruise);
        m_leftDM.configMotionAcceleration(0, accel);

        m_rightDM.config_kP(0, kp);
        m_rightDM.config_kI(0, ki);
        m_rightDM.config_kD(0, kd);
        m_rightDM.config_kF(0, kf);
        m_rightDM.configMotionCruiseVelocity(0, cruise);
        m_rightDM.configMotionAcceleration(0, accel);
        
    
    }

    /** Used to grab a singleton instance of the Drivetrain that is syncronized. */
    public static synchronized Drivetrain getInstance(){
        if (m_drivetrain == null)
            m_drivetrain = new Drivetrain();

        return m_drivetrain;     
    }

    public double scale(double input, double deadzone){

        double value = 0;
        if(input > deadzone){
            value = (input - deadzone)*(1/(1-deadzone));
        }else if(input < -deadzone){
            value = (input + deadzone)*(1/(1-deadzone));
        }else{
            value = 0;
        }

        return value;
    }

    double outputT = 0;
    double outputD = 0;
    double ramp = 0.1;
    public void arcadeDrive(double speed, double turn){

        
        if(speed > 0){
            if(speed > 0.2){
                speed = speed - 0.2;
            }else{
                speed = 0;
            }
        }else{
            if(speed < -0.2){
                speed = speed + 0.2;
            }else{
                speed = 0;
            }
        }

        if(turn > 0){
            if(turn > 0.2){
                turn = turn - 0.2;
            }else{
                turn = 0;
            }
        }else{
            if(turn < -0.2){
                turn = turn + 0.2;
            }else{
                turn = 0;
            }
        }

        outputD = outputD + (outputD - speed) * -ramp;
        double left = bound(-1,1, outputD-turn);
        double right = bound(-1,1,outputD+turn);

        m_rightDM.set(ControlMode.PercentOutput, right*m_elevator.maximumDriveSpeed());
        m_leftDM.set(ControlMode.PercentOutput, -left*m_elevator.maximumDriveSpeed());

        

        c_visionLoop.reset();
    }

    /** Autonomus driving function. */
    double outputL = 0;
    double outputR = 0;
    public void tank(double leftSpeed, double rightSpeed){

        outputL = outputL + (outputL - leftSpeed) * -ramp;
        outputR = outputR + (outputR - rightSpeed) * -ramp;

        m_rightDM.set(ControlMode.PercentOutput, outputR*m_elevator.maximumDriveSpeed());
        m_leftDM.set(ControlMode.PercentOutput, -outputL*m_elevator.maximumDriveSpeed());

        c_visionLoop.reset();
    }

    // _____AUTO FUNCTIONS_____   
    /** Drives to a distance while maintaning an angle */
    public boolean driveTo(double inches, double angle){

        inches = inches * m_magicNumber;
        double turn = c_visionLoop.getOutput(getAngle(), angle);
        double speed = 0.75;

        if(Math.abs(m_drivetrain.getEncoder()) > inches - 30){
            speed = 0.25;
        }

        

        if(Math.abs(m_drivetrain.getEncoder()) > inches - 1){
            tank(0,0);
            return true;
        }
        return false;
    }

    /** Turns the robot to an angle */
    public boolean turnTo(double angle){
        double turn = c_visionLoop.getOutput(getAngle(), angle);
        

        if(getAngle() > angle - 1 && getAngle() < angle + 1){
            tank(0,0);
            return true;
        }
        return false;
    }

    /** Drives the robot straight without incorperating a distance */    
    public void driveStraight(double angle, double speed){
        double turn = c_visionLoop.getOutput(getAngle(), angle);

       
    }

    /** Drives towards a vision target */
    public void target(){
        double target = m_vision.getTargetLocation();

        double speed = c_visionLoop.getOutput(target, 0);

        System.out.println(outputD);
        double left = bound(-1,1, -speed);
        double right = bound(-1,1,+speed);

        m_rightDM.set(ControlMode.PercentOutput, right*m_elevator.maximumDriveSpeed());
        m_leftDM.set(ControlMode.PercentOutput, -left*m_elevator.maximumDriveSpeed());

    }

    /** Returns the velocity of the drive motors */
    public double getRPM(){
        return Math.abs(m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity())/2;
    }

    /** Resets the encoders */
    public void resetEncoders(){
        m_rightEncoderOffset = (int)m_rightEncoder.getPosition();
        m_leftEncoderOffset = (int)m_leftEncoder.getPosition();
    }

    /** Resets the gyro */
    public void resetNavX(){
        m_navx.reset();
    }

    /** Returns the angle of the robot */
    public double getAngle(){
        return m_navx.getAngle();
    }

    /** Returns the left encoder value. */
    public double getLeftEncoder(){
        return (m_leftEncoder.getPosition()-m_leftEncoderOffset);
    }

    /** Returns the right encoder value. */
    public double getRightEncoder(){
        return (m_rightEncoder.getPosition()-m_rightEncoderOffset);
    }   
    
    /**  Returns the average left and right encoder values */
    public double getEncoder(){
        return Math.round((getRightEncoder() - getLeftEncoder()) / 2);
    }

    // _____MATH FUNCTIONS_____
    /** Deadzones an input */
    private double deadzone(double input, double deadzone){     
        if(input > -deadzone && input < deadzone)
            return 0;
        return input;
    }

    /** Bounds the input based on custom bounding logic. */
    private double bound(double lowerBound, double upperBound, double input){
        if(input > upperBound)
            return upperBound;
        else if(input < lowerBound)
            return lowerBound;

        return input;
    }
}