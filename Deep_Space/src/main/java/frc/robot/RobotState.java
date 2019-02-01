package frc.robot;

public class RobotState{

    private static RobotState m_rs;

    public State m_state = null;
    
    /** Syncronized Signleton creator. */
    public static synchronized RobotState getInstance(){
        if (m_rs == null)
            m_rs = new RobotState();
        return m_rs;
    }

    public void setState(State state){
        m_state = state;
    }

    public State state(){
        return m_state;
    }


    public enum State{
        GRAB_CARGO, GRAB_HATCH, PLACE_PANEL, CARGO_CS, CARGO_ROCKET, PASSTHROUGH;
    }
}