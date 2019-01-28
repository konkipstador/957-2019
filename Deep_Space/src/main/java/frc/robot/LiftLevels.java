package frc.robot;

public enum LiftLevels{
    
    // Levels of the hatch ports
    HATCH_LOW(0), HATCH_MEDIUM(0), HATCH_HIGH(0),
    // Levels of the cargo ports
    PORT_LOW(0), PORT_CARGO_SHIP(0), PORT_MEDIUM(0), PORT_HIGH(0),
    // Other Levels
    GROUND(0);
    
    // Placeholder variables for the Enumerator structure
    private final int m_encoderPosition;

    // Enum structure constructor
    private LiftLevels(int encoderPosition) { 
        m_encoderPosition = encoderPosition;
    } 

    // Get the elevator level that is target
    public int encoderPosition() 
    { 
        return m_encoderPosition;
    } 
}
