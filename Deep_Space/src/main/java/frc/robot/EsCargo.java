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

    WPI_TalonSRX m_grabbing = new WPI_TalonSRX(7);
    WPI_TalonSRX m_shooting1 = new WPI_TalonSRX(9);
    WPI_TalonSRX m_shooting2 = new WPI_TalonSRX(8);
    TalonSRX m_arm = new TalonSRX(6);

    DigitalInput m_breakBeam1 = new DigitalInput(2);
    DigitalInput m_breakBeam2 = new DigitalInput(1);
    DigitalInput m_breakBeam3 = new DigitalInput(0);

    boolean ps = false;

    private static EsCargo m_cargo;
    
    /** Syncronized Signleton creator. */
    public static synchronized EsCargo getInstance(){
        if (m_cargo == null)
            m_cargo = new EsCargo();
        return m_cargo;
    }

    public void drive(double input){
        m_arm.set(ControlMode.PercentOutput, input);
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
    }

    public void run(){      
        
        if(m_robotState.state() == State.GRAB_CARGO){
            grabbing();
            
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
            m_grabbing.set(0);

            if(!ps){
                m_shooting1.set(0);
                m_shooting2.set(0);
            }else{
                ps = false;
            }
            
        }

       m_arm.set(ControlMode.MotionMagic,m_armState.getPosition());
    }

    public void runPassthrough(double input){
        m_grabbing.set(-1*input);
        m_shooting1.set(1*input);
        m_shooting2.set(-1*input);
    }

    private void grabbing(){
        m_armState = Position.DOWN;
        m_grabbing.set(1);
    }

    private void inPassthrough(){
        m_armState = Position.UP;
        m_grabbing.set(1);
    }

    private void alignCargo(){
        m_shooting1.set(-0.15);
        m_shooting2.set(0.15);
    }

    public void placeCargo(){
        ps = true;
        m_shooting1.set(-0.5);
        m_shooting2.set(0.5);
    }

    public void stop(){
        m_shooting1.set(0);
        m_shooting2.set(0);
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
        UP(0), DOWN(-21250);

        int tickValue;

        private Position(int tickValue){
            this.tickValue = tickValue;
        }

        public int getPosition(){
            return tickValue;
        }
    }
}