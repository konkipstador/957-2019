package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI.Port;

/** 
 * Class to operate the drivetrain of the robot.
 * 
 * Automatically creates 4 Spark Max objects and master/slaves them together. Has built
 * in arcade drive features and direct commands to set drivetrain speeds, useful for
 * Auto.
 */
public class Drivetrain {

    AHRS m_navx = new AHRS(Port.kMXP);

    CANSparkMax m_rightNeoM = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_rightNeoS = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_rightEncoder = m_rightNeoM.getEncoder();

    CANSparkMax m_leftNeoM = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_leftNeoS = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_leftEncoder = m_leftNeoM.getEncoder();

    private static Drivetrain m_drivetrain = null;

    private static final int k_freeCurrentLimit = 30;
    private static final int k_stallCurrentLimit = 30;

    private int m_rightEncoderOffset = 0;
    private int m_leftEncoderOffset = 0;

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
        m_navx.reset();
    }

    public double getAngle(){
        return m_navx.getAngle();
    }

    /** Set the speeds of the left motors. */
    public void setLeft(double leftSpeed){
        m_leftNeoM.set(leftSpeed);
    }

    /** Set the speeds of the right motors. */
    public void setRight(double rightSpeed){
        m_rightNeoM.set(rightSpeed);
    }

    /** Arcade Drive command for TeleOp driving. */
    public void arcadeDrive(double speed, double turn){
        setLeft(speed+turn);
        setRight(speed-turn);
    }

    /** Returns the left encoder value. */
    public int getLeftEncoder(){
        return (int)m_leftEncoder.getPosition()-m_leftEncoderOffset;
    }

    /** Returns the right encoder value. */
    public int getRightEncoder(){
        return (int)m_rightEncoder.getPosition()-m_rightEncoderOffset;
    }   
    
    /**  Returns the average left and right encoder values */
    public double getEncoder(){
        return Math.round((getRightEncoder() + getLeftEncoder()) / 2);
    }

    public double[] getTemp(){
        return new double[]{m_leftNeoM.getMotorTemperature()};
    }
}