package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libraries.MiniPID;

/**
 * Class to operate the drivetrain of the robot.
 * 
 * Automatically creates 4 Spark Max objects and master/slaves them together.
 * Has built in arcade drive features and direct commands to set drivetrain
 * speeds, useful for Auto.
 */
public class Drivetrain {

    //AHRS m_navx = new AHRS(Port.kMXP);
    Vision m_vision = Vision.getInstance();
    Elevator m_elevator = Elevator.getInstance();

    public double vkP = 5e-5,
    vkI = 1e-6,
    vkD = 0,
    vkIz = 0, 
    vkFF = 0, 
    vkMaxOutput = 1, 
    vkMinOutput = -1;

    CANSparkMax m_rightNeoM = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_rightNeoS = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_rightEncoder = m_rightNeoM.getEncoder();
    private CANPIDController m_rightVelocity = m_rightNeoM.getPIDController();
    
    CANSparkMax m_leftNeoM = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_leftNeoS = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_leftEncoder = m_leftNeoM.getEncoder();
    private CANPIDController m_leftVelocity = m_leftNeoM.getPIDController();

    private static Drivetrain m_drivetrain = null;

    private static final int k_freeCurrentLimit = 40;
    private static final int k_stallCurrentLimit = 40;

    double m_rightEncoderOffset = 0;
    double m_leftEncoderOffset = 0;

    /**
     * Drivetrain constructor, which is called automatically when an instance of the
     * drivetrain is asked for. You should not call directly.
     */
    public Drivetrain(){
        m_rightNeoS.follow(m_rightNeoM);
        m_leftNeoS.follow(m_leftNeoM);

        // Neo Current Limits
        m_rightNeoM.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_rightNeoS.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_leftNeoM.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_leftNeoS.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_rightNeoM.setRampRate(0);
        m_rightNeoS.setRampRate(0);
        m_leftNeoM.setRampRate(0);
        m_leftNeoS.setRampRate(0);

        m_rightVelocity.setP(vkP);
        m_rightVelocity.setI(vkI);
        m_rightVelocity.setD(vkD);
        m_rightVelocity.setIZone(vkIz);
        m_rightVelocity.setFF(vkFF);
        m_rightVelocity.setOutputRange(vkMinOutput, vkMaxOutput);

        m_leftVelocity.setP(vkP);
        m_leftVelocity.setI(vkI);
        m_leftVelocity.setD(vkD);
        m_leftVelocity.setIZone(vkIz);
        m_leftVelocity.setFF(vkFF);
        m_leftVelocity.setOutputRange(vkMinOutput, vkMaxOutput);
    
    }

    /** Used to grabe a singleton instance of the Drivetrain that is syncronized. */
    public static synchronized Drivetrain getInstance(){
        if (m_drivetrain == null)
            m_drivetrain = new Drivetrain();

        return m_drivetrain;     
    }

    public void resetEncoders(){
        m_rightEncoderOffset = (int)m_rightEncoder.getPosition();
        m_leftEncoderOffset = (int)m_leftEncoder.getPosition();
    }

    public void resetNavX(){
        //m_navx.reset();
    }

    public double getAngle(){
        return 0;
    }

    /** Set the speeds of the left motors. */
    public void tank(double leftSpeed, double rightSpeed){

        double deadzoneLeft = deadzone(leftSpeed, 0.5);
        double deadzoneRight = deadzone(rightSpeed, 0.5);

        double left = bound(-1,1, deadzoneLeft);
        double right = bound(-1,1,deadzoneRight);

        m_rightVelocity.setReference(right*m_elevator.maximumDriveSpeed(), ControlType.kVelocity);
        m_leftVelocity.setReference(-left*m_elevator.maximumDriveSpeed(), ControlType.kVelocity);

        c_visionLoop.reset();
    }

    private double deadzone(double input, double deadzone){     
        if(input > -deadzone && input < deadzone)
            return 0;
        return input;
    }

    /** Arcade Drive command for TeleOp driving. */
    public void arcadeDrive(double speed, double turn){
        speed = speed-0.05;

        double deadzoneSpeed = deadzone(speed, 0.5);
        double deadzoneTurn = deadzone(turn, 0.5);

        double left = bound(-1,1, deadzoneSpeed+deadzoneTurn);
        double right = bound(-1,1,deadzoneSpeed+deadzoneTurn);

        m_rightVelocity.setReference(right*m_elevator.maximumDriveSpeed(), ControlType.kVelocity);
        m_leftVelocity.setReference(-left*m_elevator.maximumDriveSpeed(), ControlType.kVelocity);

        c_visionLoop.reset();
    }

    double v_vkp = 0.02;
    double v_vki = 0.;
    double v_vkd = 0;
    MiniPID c_visionLoop = new MiniPID(v_vkp,v_vki,v_vki);

    public void target(){
        double target = m_vision.getTargetLocation();

        double speed = c_visionLoop.getOutput(target, 0);

        m_rightVelocity.setReference((-speed-0.2)*2400, ControlType.kVelocity);
        m_leftVelocity.setReference((-speed+0.2)*2400, ControlType.kVelocity);
    }

    /** Bounds the input based on custom bounding logic. */
    private double bound(double lowerBound, double upperBound, double input){
        if(input > upperBound)
            return upperBound;
        else if(input < lowerBound)
            return lowerBound;

        return input;
    }

    /** Returns the left encoder value. */
    public int getLeftEncoder(){
        return (int)(m_leftEncoder.getPosition()-m_leftEncoderOffset);
    }

    /** Returns the right encoder value. */
    public int getRightEncoder(){
        return (int)(m_rightEncoder.getPosition()-m_rightEncoderOffset);
    }   
    
    /**  Returns the average left and right encoder values */
    public double getEncoder(){
        return Math.round((getRightEncoder() - getLeftEncoder()) / 2);
    }

    public double[] getTemp(){
        return new double[]{m_leftNeoM.getMotorTemperature(), m_rightNeoM.getMotorTemperature()};      
    }
}