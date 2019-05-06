package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Elevator.LiftLevels;
import frc.robot.RobotState.State;

public class CargoSystem{

    RobotState m_state = RobotState.getInstance();
    Elevator m_elevator = Elevator.getInstance();

    DoubleSolenoid m_arm = new DoubleSolenoid(12,4,5);
    WPI_TalonSRX m_passthrough = new WPI_TalonSRX(7);
    WPI_TalonSRX m_shooterL = new WPI_TalonSRX(8);
    WPI_TalonSRX m_shooterR = new WPI_TalonSRX(9);

    DigitalInput m_frontShooterSensor = new DigitalInput(0);
    DigitalInput m_backShooterSensor = new DigitalInput(1);
    DigitalInput m_passthroughSensor = new DigitalInput(2);
    
    boolean m_overrideArmState = false;
    double m_overridePassthroughState = 0;
    double m_manualShooterSpeed = 0;

    private static CargoSystem m_cargoSystem = null;

    private CargoSystem(){
        // Right shooter motor follows the left and is inverted.
        m_shooterR.setInverted(true);
        m_shooterR.follow(m_shooterL);
    }

    /** Used to grab a singleton instance of the Drivetrain that is syncronized. */
    public static synchronized CargoSystem getInstance(){
        if (m_cargoSystem == null)
            m_cargoSystem = new CargoSystem();

        return m_cargoSystem;     
    }

    public void run(){
        boolean armState = false;
        double shooterSpeed = 0;
        double passthroughSpeed = 0;

        /** Check robot state to calculate base system control prior to checking overrides */
        switch(m_state.state()){

            case GRAB_CARGO:

                armState = true;
                passthroughSpeed = 1;
                shooterSpeed = .22;

                if(!m_passthroughSensor.get() || !m_frontShooterSensor.get() || !m_backShooterSensor.get()){
                    m_state.setState(State.PASSTHROUGH);
                }

                m_elevator.setLevel(LiftLevels.HATCH_LOW);

                break;

            case GRAB_CARGO_FEEDER:

                armState = false;
                passthroughSpeed = 1;
                shooterSpeed = .22;

                if(!m_passthroughSensor.get() || !m_frontShooterSensor.get() || !m_backShooterSensor.get()){
                    m_state.setState(State.PASSTHROUGH);
                }

                m_elevator.setLevel(LiftLevels.HATCH_LOW);

                break;

            case PASSTHROUGH:

                armState = false;
                passthroughSpeed = 1;
                shooterSpeed = 0.2;

                if(!m_frontShooterSensor.get() && m_backShooterSensor.get()){
                    m_state.setState(State.PLACE_CARGO);
                }

                break;

            case PLACE_CARGO:

                armState = false;
                passthroughSpeed = 0;

                if(m_frontShooterSensor.get()){
                    m_state.setState(State.PLACE_PANEL);
                }

                break;

            default:

                armState = false;
                passthroughSpeed = 0;
                shooterSpeed = 0;

                break;
        }

        /** Checks for individual overrides of system functions. If no override is detected,
         *  the system is ran as normal. If one is detected, the state of the sub-mechanism
         *  is set to the override, excepting the arm. The override is reset to false upon
         *  checking each state. */
        if(m_overrideArmState){
            armState = false;
            m_overrideArmState = false;
        }

        if(m_overridePassthroughState != 0){
            passthroughSpeed = m_overridePassthroughState;
            m_overridePassthroughState = 0;
        }

        if(m_manualShooterSpeed != 0){
            shooterSpeed = m_manualShooterSpeed;
            m_manualShooterSpeed = 0;
        }

        if(armState){
            m_arm.set(Value.kReverse);
        }else{
            m_arm.set(Value.kForward);
        }

        
        m_passthrough.set(passthroughSpeed);
        m_shooterL.set(shooterSpeed);
    }

    public void shoot(double speed){
        m_manualShooterSpeed = speed;
    }

    public void runSystem(){
        m_overrideArmState = true;
        m_overridePassthroughState = 1;
        m_manualShooterSpeed = 0.15;
    }

    public void runPassthrough(double speed){
        m_overridePassthroughState = speed;
        m_manualShooterSpeed = speed/6;
    }

    public boolean getPassthroughSensor(){
        return m_passthroughSensor.get();
    }

    public boolean getBackShooterSensor(){
        return m_backShooterSensor.get();
    }

    public boolean getFrontShooterSensor(){
        return m_frontShooterSensor.get();
    }

    public double getCurrent(){
        return m_passthrough.getOutputCurrent() + m_shooterL.getOutputCurrent() + m_shooterR.getOutputCurrent();
    }
}
