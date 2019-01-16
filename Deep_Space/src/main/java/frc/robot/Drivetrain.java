package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/** 
 * Class to operate the drivetrain of the robot.
 * 
 * Automatically creates 4 Spark Max objects and master/slaves them together. Has built
 * in arcade drive features and direct commands to set drivetrain speeds, useful for
 * Auto.
 */
public class Drivetrain {

    CANSparkMax m_rightNeoM = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_rightNeoS = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_rightEncoder = m_rightNeoM.getEncoder();

    CANSparkMax m_leftNeoM = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_leftNeoS = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_leftEncoder = m_leftNeoM.getEncoder();

    private static Drivetrain m_drivetrain = null;

    /**
     * Drivetrain constructor, which is called automatically when an instance of the
     * drivetrain is asked for. Do not call directly.
     */
    public Drivetrain(){
        m_rightNeoS.follow(m_rightNeoM);
        m_leftNeoS.follow(m_leftNeoM);
    }

    /** Used to grabe a singleton instance of the Drivetrain that is syncronized. */
    public static synchronized Drivetrain getInstance(){
        if (m_drivetrain == null)
            m_drivetrain = new Drivetrain();

        return m_drivetrain;     
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
    public double getLeftEncoder(){
        return m_leftEncoder.getPosition();
    }

    /** Returns the right encoder value. */
    public double getRightEncoder(){
        return m_rightEncoder.getPosition();
    }   
    
    /**  Returns the average left and right encoder values */
    public double getEncoder(){
        return Math.round((getRightEncoder() + getLeftEncoder()) / 2);
    }
}