package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotState.State;

public class EsCargo {

    RobotState m_robotState = RobotState.getInstance();

    WPI_TalonSRX m_grabbing = new WPI_TalonSRX(6);
    WPI_TalonSRX m_shooting1 = new WPI_TalonSRX(7);
    WPI_TalonSRX m_shooting2 = new WPI_TalonSRX(8);

    Solenoid m_arm1 = new Solenoid(3);
    Solenoid m_arm2 = new Solenoid(4);

    boolean m_armState = false;

    DigitalInput m_breakBeam1 = new DigitalInput(1);
    DigitalInput m_breakBeam2 = new DigitalInput(2);

    State m_lastTarget = State.CARGO_ROCKET;

    public EsCargo(){

    }

    public void run(){
        
        if(m_robotState.state() == State.GRAB_CARGO){
            grabbing();
            
            if(!m_breakBeam1.get()){
                m_robotState.setState(State.PASSTHROUGH);
            }
        }
        
        if(m_robotState.state() == State.PASSTHROUGH){
            inPassthrough();
            
            if(!m_breakBeam2.get()){
                m_robotState.setState(State.CARGO_ROCKET);
            }
        }

        if(m_robotState.state() == State.CARGO_ROCKET || m_robotState.state() == State.CARGO_CS){

            m_grabbing.set(0);

            if(m_breakBeam2.get()){
                m_robotState.setState(State.GRAB_HATCH);
            }
        }

        if(m_robotState.state() == State.CARGO_ROCKET){
            m_lastTarget = State.CARGO_ROCKET;
        }
    }

    private void grabbing(){
        m_arm1.set(true);
        m_arm2.set(true);
        m_grabbing.set(1);
    }

    private void inPassthrough(){
        m_arm1.set(false);
        m_arm2.set(false);
        m_grabbing.set(1);
    }

    public void placeCargo(){
        m_shooting1.set(0.5);
        m_shooting2.set(0.5);
    }

    public void stop(){
        m_shooting1.set(0);
        m_shooting2.set(0);
    }
}