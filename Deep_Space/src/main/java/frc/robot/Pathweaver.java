package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Pathweaver {

    public static Pathweaver m_pathweaver;
    public Drivetrain m_drivetrain = Drivetrain.getInstance();  // Grabbing encoder values from drivetrain

    // PATHFINDER CONSTANTS
    private static final int k_ticks_per_rev = 365;
    private static final double k_wheel_diameter = 6/12;
    private static final double k_max_velocity = 5;
    private static final double kp = 1;
    private static final double ki = 0;
    private static final double kd = 0;
    private static final double ka = 0;

    private ArrayList<EncoderFollower> m_left_followers;
    private ArrayList<EncoderFollower> m_right_followers;

    private ArrayList<Trajectory> m_left_trajectories;
    private ArrayList<Trajectory> m_right_trajectories;

    /** Syncronized Signleton creator. */
    public static synchronized Pathweaver getInstance(){
        if (m_pathweaver == null)
            m_pathweaver = new Pathweaver();

        return m_pathweaver;
    }

    /** Overwrites previous loaded paths and generates new ones strung from Path objects. */
    public void loadPath(Path[] paths){

        m_left_followers = new ArrayList<EncoderFollower>();
        m_right_followers = new ArrayList<EncoderFollower>();
        m_left_trajectories = new ArrayList<Trajectory>();
        m_right_trajectories = new ArrayList<Trajectory>();

        int pathCount = 0;

        // Compile EncoderFollowers
        for(Path path : paths){
            // As of 1-25-19, PathWeaver swaps the left and right paths.
            m_left_trajectories.add(PathfinderFRC.getTrajectory(path.getName() + ".right"));
            m_right_trajectories.add(PathfinderFRC.getTrajectory(path.getName() + ".left"));

            System.out.println("Loaded path "+ path.getName());
            // Left Followers
            EncoderFollower follower = new EncoderFollower(m_left_trajectories.get(pathCount));
            follower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
            follower.configurePIDVA(kp, ki, kd, 1/k_max_velocity, ka);
            m_left_followers.add(follower);

            // Right Followers
            follower = new EncoderFollower(m_right_trajectories.get(pathCount));
            follower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
            follower.configurePIDVA(kp, ki, kd, 1/k_max_velocity, ka);
            m_right_followers.add(follower);

            System.out.println("Loaded Encoder Followers for "+ path.getName());

            pathCount++;
        }
    }

    // TODO: 
    /** Runs the indicated path. Returns true if the path is completed, returns false otherwise. */
    public boolean runPath(int pathNumber){
        if(m_right_followers.get(pathNumber).isFinished()){
            m_drivetrain.tank(0,0);
            return true;
        }

        double leftPower = m_left_followers.get(pathNumber).calculate(-m_drivetrain.getLeftEncoder());
        double rightPower = m_right_followers.get(pathNumber).calculate(m_drivetrain.getRightEncoder());

        double heading = m_drivetrain.getAngle();
        double desired_heading = Pathfinder.r2d(m_left_followers.get(pathNumber).getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn =  0.8 * (-1.0/80.0) * heading_difference;

        System.out.println();
        SmartDashboard.putNumber("encoder",m_drivetrain.getLeftEncoder());
        SmartDashboard.putNumber("angle",desired_heading);
        m_drivetrain.tank(leftPower,rightPower);
        return false;
    }

    /** Enum to promote readability of auto paths. */
    public enum Path{
        ROCKET_RIGHT_1("RightRocket1"), ROCKET_RIGHT_2("RightRocket2"), ROCKET_RIGHT_3("RightRocket3");

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