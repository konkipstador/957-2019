package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Class to operate the drivetrain of the robot.
 * 
 * Automatically creates 4 Spark Max objects and master/slaves them together. Has built
 * in arcade drive features and direct commands to set drivetrain speeds, useful for
 * Auto.
 */
public class Drivetrain {

    //AHRS m_navx = new AHRS(Port.kMXP);

    double kp = 0.0001;
    double ki = 0;
    double kd = 0;

    CANSparkMax m_rightNeoM = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_rightNeoS = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_rightEncoder = m_rightNeoM.getEncoder();
    MiniPID c_right = new MiniPID(kp,ki,kd);

    CANSparkMax m_leftNeoM = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_leftNeoS = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_leftEncoder = m_leftNeoM.getEncoder();
    MiniPID c_left = new MiniPID(kp,ki,kd);

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
        //m_rightNeoM.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        //m_rightNeoS.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        //m_leftNeoM.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        //m_leftNeoS.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        //m_rightNeoM.setRampRate(0);
        //m_rightNeoS.setRampRate(0);
        //m_leftNeoM.setRampRate(0);
        //m_leftNeoS.setRampRate(0);
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
        c_left.reset();
        c_right.reset();
    }

    public void resetNavX(){
        //m_navx.reset();
    }

    public double getAngle(){
        return 0;
    }

    /** Set the speeds of the left motors. */
    public void setLeft(double leftSpeed){
        m_leftNeoM.set(-leftSpeed);
    }

    /** Set the speeds of the right motors. */
    public void setRight(double rightSpeed){
        m_rightNeoM.set(rightSpeed);
    }

    /** Arcade Drive command for TeleOp driving. */
    public void arcadeDrive(double speed, double turn){
        if(speed > -.1 && speed < .1)
            speed = 0;
        if(turn > -.1 && turn < .1)
            turn = 0;

        double left = bound(-1,1, speed);
        double right = bound(-1,1,speed);

        SmartDashboard.putNumber("leftValue", left);
        SmartDashboard.putNumber("rightValue", right);

        
        double leftSpeed = c_left.getOutput(m_leftEncoder.getVelocity(), left*6000);
        double rightSpeed = c_right.getOutput(m_rightEncoder.getVelocity(), right*6000);

        SmartDashboard.putNumber("leftSpeed", leftSpeed);
        SmartDashboard.putNumber("rightSpeed", rightSpeed);


        setLeft(leftSpeed);
        setRight(rightSpeed);
        SmartDashboard.putNumber("rpm l", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("rpm r", m_rightEncoder.getVelocity());
    }

    private double bound(double lowerBound, double upperBound, double input){
        if(input > upperBound)
            return upperBound;
        else if(input < lowerBound)
            return lowerBound;

        return input;
    }

    /** Returns the left encoder value. */
    public int getLeftEncoder(){
        return (int)(m_leftEncoder.getPosition()+m_leftEncoderOffset);
    }

    /** Returns the right encoder value. */
    public int getRightEncoder(){
        return (int)(m_rightEncoder.getPosition()-m_rightEncoderOffset);
    }   
    
    /**  Returns the average left and right encoder values */
    public double getEncoder(){
        return Math.round((getRightEncoder() + getLeftEncoder()) / 2);
    }

    public double[] getTemp(){
        SmartDashboard.putNumber("Power Draw R", m_rightNeoM.getOutputCurrent());
        SmartDashboard.putNumber("Power Draw L", m_leftNeoM.getOutputCurrent());
        return new double[]{m_leftNeoM.getMotorTemperature(), m_rightNeoM.getMotorTemperature()};
        
    }
}