package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Pathweaver {


    public static Pathweaver m_pathweaver;

    // Encoder values are drawn from the drivetrain
    public Drivetrain m_drivetrain = Drivetrain.getInstance();

    private static final int k_ticks_per_rev = 1024;
    private static final double k_wheel_diameter = 4.0 / 12.0;
    private static final double k_max_velocity = 10;

    private static final int k_left_channel = 0;
    private static final int k_right_channel = 0;

    private ArrayList<EncoderFollower> m_left_followers;
    private ArrayList<EncoderFollower> m_right_followers;

    private ArrayList<Trajectory> m_left_trajectories;
    private ArrayList<Trajectory> m_right_trajectories;

    private Notifier m_notifier;

    /** Syncronized Signleton creator. */
    public static synchronized Pathweaver getInstance(){
        if (m_pathweaver == null)
            m_pathweaver = new Pathweaver();

        return m_pathweaver;
    }

    public void loadPath(Path[] paths){
        ArrayList<Path> pathArray = new ArrayList<Path>(Arrays.asList(paths));

        m_left_followers = new ArrayList<EncoderFollower>();
        m_right_followers = new ArrayList<EncoderFollower>();
        m_left_trajectories = new ArrayList<Trajectory>();
        m_right_trajectories = new ArrayList<Trajectory>();

        for(Path path : paths){
            m_left_trajectories.add(PathfinderFRC.getTrajectory(path.getName() + ".left"));
            m_left_trajectories.add(PathfinderFRC.getTrajectory(path.getName() + ".right"));
        }
    }

    public enum Path{
        ROCKET_RIGHT_1("RocketRight1"), ROCKET_RIGHT_2("RocketRight2"), ROCKET_RIGHT_3("RocketRight3");

        // Placeholder variables for the Enumerator structure
        private final String m_pathName;

        // Enum structure constructor
        private Path(String pathName) { 
            m_pathName = pathName;
        } 

        // Get the elevator level that is target
        public String getName() 
        { 
            return m_pathName;
        } 
    }
}