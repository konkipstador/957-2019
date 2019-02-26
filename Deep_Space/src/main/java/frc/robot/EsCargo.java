package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.libraries.MiniPID;
import frc.robot.RobotState.State;

public class EsCargo {
    
    double kp = 1.27;
    double ki = 0;
    double kd = 0;
    double kf = 0.682;

    Position m_armState = Position.UP;

    RobotState m_robotState = RobotState.getInstance();

    TalonSRX m_arm = new TalonSRX(6);
    WPI_TalonSRX m_grabbing = new WPI_TalonSRX(7);
    WPI_TalonSRX m_shootingR = new WPI_TalonSRX(9);
    WPI_TalonSRX m_shootingL = new WPI_TalonSRX(8);
    

    DigitalInput m_breakBeam1 = new DigitalInput(2);
    DigitalInput m_breakBeam2 = new DigitalInput(1);
    DigitalInput m_breakBeam3 = new DigitalInput(0);
  
    boolean ps = false;
    boolean m_invertArm = false;
    boolean grabbing = false;

    private static EsCargo m_cargo;
    
    /** Syncronized Signleton creator. */
    public static synchronized EsCargo getInstance(){
        if (m_cargo == null)
            m_cargo = new EsCargo();
        return m_cargo;
    }

    public EsCargo(){
        m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);     
        m_arm.setSelectedSensorPosition(0);
        m_arm.setSensorPhase(false);
        m_arm.config_kP(0, kp);
        m_arm.config_kI(0, ki);
        m_arm.config_kD(0, kd);
        m_arm.config_kF(0, kf);
        m_arm.configMotionCruiseVelocity(1500);
        m_arm.configMotionAcceleration(3000);  
        m_grabbing.configContinuousCurrentLimit(10);
        //m_grabbing.configPeakCurrentLimit(10);
        //m_grabbing.configPeakCurrentDuration(10);
        //m_grabbing.enableCurrentLimit(true);
    }

    public void run(){
        
        if(m_robotState.state() == State.GRAB_CARGO){
            grabbing();
            alignCargo();

            
            if(!m_breakBeam1.get()){
                m_robotState.setState(State.PASSTHROUGH);
            }
        }else if(m_robotState.state() == State.PASSTHROUGH){
            inPassthrough();
            alignCargo();

            
            if(!m_breakBeam2.get()){
                m_robotState.setState(State.CARGO_ALIGNMENT);
            }
        }else if(m_robotState.state() == State.CARGO_ALIGNMENT){
            alignCargo();
            inPassthrough();
            
            if(m_breakBeam2.get() && !m_breakBeam3.get()){
                m_robotState.setState(State.PLACE_CARGO);
                stop();
            }
        }else if(m_robotState.state() == State.PLACE_CARGO){
            
            m_grabbing.set(0);

            if(m_breakBeam3.get()){

                m_robotState.setState(State.PLACE_PANEL);
            }
        }else{
            m_armState = Position.UP;
            if(m_invertArm == true){
                m_armState = Position.DOWN;
                m_invertArm = false;
            }
            
            if(!grabbing){
                if(reverse){
                    m_grabbing.set(-1);
                }else{
                    m_grabbing.set(0);
                }
                reverse = false;
                
            }else{
                if(reverse){
                    m_grabbing.set(-1);
                }
                grabbing = false;
                reverse = false;
            }

            if(!ps){
                m_shootingR.set(0);
                m_shootingL.set(0);
                
            }else{
                ps = false;
            }

            
            
        }

       m_arm.set(ControlMode.MotionMagic,m_armState.getPosition());
    }

    boolean reverse = false;
    public void reverse(){
        reverse = true;
        if(m_robotState.state() == State.GRAB_CARGO || m_robotState.state() == State.PASSTHROUGH){
            m_robotState.setState(State.PLACE_PANEL);
        }
    }

    public void lowerArm(){
        m_invertArm = true;
    }

    public void runPassthrough(double input){
        grabbing = true;
        m_grabbing.set(1*input);
        m_shootingR.set(1*input);
        m_shootingL.set(-1*input);
    }

    private void grabbing(){
        m_armState = Position.DOWN;
        m_grabbing.set(1);
    }

    private void inPassthrough(){
        grabbing = true;
        m_armState = Position.UP;
        m_grabbing.set(1);
    }

    private void alignCargo(){
        m_shootingR.set(-0.15);
        m_shootingL.set(0.15);
    }

    public void placeCargo(double speed){
        ps = true;
        m_shootingR.set(-speed);
        m_shootingL.set(speed);
    }

    public void stop(){
        m_shootingR.set(0);
        m_shootingL.set(0);
    }

    public int getArmPosition(){
        return m_arm.getSelectedSensorPosition(0);
    }

    public boolean get1(){
        return m_breakBeam1.get();
    }

    public boolean get2(){
        return m_breakBeam2.get();
    }

    public boolean get3(){
        return m_breakBeam3.get();
    }

    public enum Position{
        UP(0), DOWN(-19500);

        int tickValue;

        private Position(int tickValue){
            this.tickValue = tickValue;
        }

        public int getPosition(){
            return tickValue;
        }
    }
}