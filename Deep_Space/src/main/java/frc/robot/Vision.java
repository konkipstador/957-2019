package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {

    public static Vision m_vision;
    public Elevator m_elevator = Elevator.getInstance();

    NetworkTableEntry m_limelightX;
    NetworkTableEntry m_piX1;
    NetworkTableEntry m_piX2;
    NetworkTableEntry m_piX3;
    NetworkTableEntry m_piX4;

    public Vision(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limelight = inst.getTable("limelight");
        NetworkTable pi = inst.getTable("pi");

        m_limelightX = limelight.getEntry("tx");
        m_piX1 = pi.getEntry("px1");
        m_piX2 = pi.getEntry("px2");
        m_piX3 = pi.getEntry("px3");
        m_piX4 = pi.getEntry("px4");
    }

    public double getTargetLocation(){
        SmartDashboard.putNumber("target", m_limelightX.getDouble(0));
        return m_limelightX.getDouble(0);
    }

    /** Syncronized Signleton creator. */
    public static synchronized Vision getInstance(){
        if (m_vision == null)
            m_vision = new Vision();
        return m_vision;
    }
}
 
 
 
 
