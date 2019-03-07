package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    public static Vision m_vision;
    public Elevator m_elevator = Elevator.getInstance();

    NetworkTableEntry m_limelightX;
    NetworkTableEntry m_piX1;
    NetworkTableEntry m_piX2;
    NetworkTableEntry m_piX3;
    NetworkTableEntry m_piX4;

    NetworkTableEntry m_camMode;

    NetworkTableEntry m_streams;

    String[] streams = {"http://10.9.57.3:5800/"};

    public Vision(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limelight = inst.getTable("limelight");
        NetworkTable pi = inst.getTable("pi");

        m_limelightX = limelight.getEntry("tx");
        m_piX1 = pi.getEntry("px1");
        m_piX2 = pi.getEntry("px2");
        m_piX3 = pi.getEntry("px3");
        m_piX4 = pi.getEntry("px4");

        m_camMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");
        m_camMode.setNumber(1);

        m_streams = NetworkTableInstance.getDefault().getTable("CameraPublisher").getSubTable("Camera").getEntry("streams");
        m_streams.setStringArray(streams);
    }

    public double getTargetLocation(){
        return m_limelightX.getDouble(0);
    }

    public void enableTracking(){
        m_camMode.setNumber(0);
    }

    public void disableTracking(){
        m_camMode.setNumber(1);
    }

    /** Syncronized Signleton creator. */
    public static synchronized Vision getInstance(){
        if (m_vision == null)
            m_vision = new Vision();
        return m_vision;
    }
}
 
 
 
 
