package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.I2C.Port;
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

    double v_vkp = 0.1;
    double v_vki = 0.;
    double v_vkd = 0;
    MiniPID c_visionLoop = new MiniPID(v_vkp,v_vki,v_vki);

    CANSparkMax m_rightNeoMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_rightNeoSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_rightEncoder = m_rightNeoMaster.getEncoder();
    
    CANSparkMax m_leftNeoMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_leftNeoSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANEncoder m_leftEncoder = m_leftNeoMaster.getEncoder();

    private static Drivetrain m_drivetrain = null;

    private static final int k_freeCurrentLimit = 40;
    private static final int k_stallCurrentLimit = 40;

    double m_rightEncoderOffset = 0;
    double m_leftEncoderOffset = 0;

    double m_magicNumber = 0;

    /**
     * Drivetrain constructor, which is called automatically when an instance of the
     * drivetrain is asked for. You should not call directly.
     */
    public Drivetrain(){
        m_rightNeoSlave.follow(m_rightNeoMaster);
        m_leftNeoSlave.follow(m_leftNeoMaster);
            
        m_rightNeoMaster.setIdleMode(IdleMode.kCoast);
        m_rightNeoSlave.setIdleMode(IdleMode.kCoast);
        m_leftNeoMaster.setIdleMode(IdleMode.kCoast);
        m_leftNeoSlave.setIdleMode(IdleMode.kCoast);
        //m_leftNeoS.setInverted(true);

        // Neo Current Limits
        m_rightNeoMaster.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_rightNeoSlave.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_leftNeoMaster.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);
        m_leftNeoSlave.setSmartCurrentLimit(k_stallCurrentLimit, k_freeCurrentLimit);

        c_visionLoop.setOutputLimits(-.5,.5);
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

        m_rightNeoMaster.set(right*m_elevator.maximumDriveSpeed());
        m_leftNeoMaster.set(-left*m_elevator.maximumDriveSpeed());

        c_visionLoop.reset();
    }

    /** Autonomus driving function. */
    double outputL = 0;
    double outputR = 0;
    double maxChange = 0.05;
    public void tank(double leftSpeed, double rightSpeed){

        double l = outputL + (outputL - leftSpeed) * -ramp;
        double r = outputR + (outputR - rightSpeed) * -ramp;

        if(l > outputL + maxChange){
            outputL = outputL + maxChange;
        }else if(l < outputL - maxChange){
            outputL = outputL - maxChange;
        }else{
            outputL = l;
        }

        if(r > outputR + maxChange){
            outputR = outputR + maxChange;
        }else if(r < outputR - maxChange){
            outputR = outputR - maxChange;
        }else{
            outputR = r;
        }

        m_rightNeoMaster.set(outputR*m_elevator.maximumDriveSpeed());
        m_leftNeoMaster.set(-outputL*m_elevator.maximumDriveSpeed());

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

        autocade(0,-turn);
        

        if(getAngle() > angle - 1 && getAngle() < angle + 1){
            tank(0,0);
            return true;
        }
        return false;
    }

    /** Drives towards a vision target */
    public boolean target(double desiredAngle){
        double target = m_vision.getTargetLocation();
        double turn = 0;
        double left = 0;

        if(m_elevator.maximumDriveSpeed() == 0.2){
            autocade(-.6,target/15*3);
        }else{
            autocade(-0.2,target/50);
        }
        
        if(m_leftEncoder.getVelocity() < 100){
            System.out.println("done");
            return true;
        }
        return false;
    }

    boolean m_angleFound = false;

    public void refreshTargetAngle(){
        m_angleFound = false;
    }

    public boolean targetAngle(double desiredAngle){
        double target = m_vision.getTargetLocation();
        
        double left = 0;

        if(target == 0 && !m_angleFound){
            double turn = 0;
            if(m_drivetrain.getAngle() > desiredAngle+2){

                if(desiredAngle < m_drivetrain.getAngle()-5){
                    turn = 0.15;
                }else{
                    turn = 0.10;
                }
        }

            if(m_drivetrain.getAngle() < desiredAngle-2){
                if(desiredAngle > m_drivetrain.getAngle()+5){
                    turn = -0.15;
                }else{
                    turn = -0.10;
                }
            }

            if(m_elevator.maximumDriveSpeed() == 0.2){
                autocade(-.6,turn);
            }else{
                autocade(-0.2,turn);
            }

            return false;
        }else{
            m_angleFound = true;
            if(m_elevator.maximumDriveSpeed() == 0.2){
                autocade(-.6,target/15*3);
            }else{
                autocade(-0.2,target/50);
            }
            
            if(m_leftEncoder.getVelocity() < 100){
                System.out.println("done");
                return true;
            }
            return false;
        }
        
    }

    public boolean faceTarget(){

        double target = m_vision.getTargetLocation();
        double turn = 0;
        double left = 0;

        if(m_elevator.maximumDriveSpeed() == 0.2){
            autocade(0,target/15*3);
        }else{
            autocade(0,target/50);
        }
        
        if(Math.abs(target) < 2){
            System.out.println("done");
            return true;
        }
        return false;

    }

    public void autocade(double speed, double turn){

        outputD = outputD + (outputD - speed) * -ramp;
        double left = bound(-1,1, outputD-turn);
        double right = bound(-1,1,outputD+turn);

        m_rightNeoMaster.set(right*m_elevator.maximumDriveSpeed());
        m_leftNeoMaster.set(-left*m_elevator.maximumDriveSpeed());

        c_visionLoop.reset();
    }

    /** Returns the velocity of the drive motors */
    public double getRPM(){
        return Math.abs(m_leftEncoder.getVelocity() + m_rightEncoder.getVelocity())/2;
    }

    /** Resets the encoders */
    public void resetEncoders(){
        m_rightEncoderOffset = (int)(m_rightEncoder.getPosition());
        m_leftEncoderOffset = (int)(m_leftEncoder.getPosition());
    }

    /** Resets the gyro */
    public void resetNavX(){
        m_navx.reset();
    }

    /** Returns the angle of the robot */
    public double getAngle(){
        return -m_navx.getAngle();
    }

    /** Returns the left encoder value. */
    public double getLeftEncoder(){
        return (int)(m_leftEncoder.getPosition()-m_leftEncoderOffset);
    }

    /** Returns the right encoder value. */
    public double getRightEncoder(){
        return (int)(m_rightEncoder.getPosition()-m_rightEncoderOffset);
    }   
    
    /**  Returns the average left and right encoder values */
    public double getEncoder(){
        return -((getRightEncoder() - getLeftEncoder()) / (2*8.68))*3.14159*5.95;    
    }

    public double getCurrent(){
        return m_leftNeoMaster.getOutputCurrent() + m_leftNeoSlave.getOutputCurrent() + 
               m_rightNeoMaster.getOutputCurrent() + m_rightNeoSlave.getOutputCurrent();
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