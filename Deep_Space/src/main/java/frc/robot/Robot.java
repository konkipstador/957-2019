/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libraries.MiniPID;
import frc.robot.Elevator.LiftLevels;
import frc.robot.RobotState.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    Drivetrain m_drivetrain = Drivetrain.getInstance();
    Elevator m_elevator = Elevator.getInstance();
    RobotState m_robotState = RobotState.getInstance();
    Vision m_vision = Vision.getInstance();
    CargoSystem m_cargoSystem = CargoSystem.getInstance();
    Joystick m_joystick = new Joystick(0);
    Joystick m_nav = new Joystick(1);
    Climber m_climber = new Climber();

    boolean m_autoState = false;

    int m_autoPhase = 0;
    int m_cylinderState = 0;
    int m_pegState = 0;
    int m_pokeState = 0;

    Auto m_autoMode = Auto.ROCKET_RIGHT;
    
    public void robotInit() {      
        m_robotState.setState(State.PLACE_PANEL);     
        m_drivetrain.resetNavX();  
        //m_climber.raiseFront();
    }

    public void autonomousInit() {
        m_drivetrain.resetEncoders();
        m_climber.lowerFront();
        m_drivetrain.resetNavX();;
        m_autoPhase = 0;
        m_autoStep = 0;
    }

    int m_autoStep = 0;
    int ticks = 0;
    int ticks2 = 0;
    double storedAngle = 0;

    public void autonomousPeriodic() {
        
        switch(m_autoMode){
            
            case ROCKET_RIGHT:

                switch(m_autoStep){
                    case 0:
                    m_elevator.retract();
                    if(driveForward(0.5, -2.5, 55,65)){
                        m_autoStep++;
                        m_elevator.setLevel(LiftLevels.HATCH_LOW);
                        m_elevator.grab();
                        m_elevator.extend();
                        ticks = 0;
                    }
        
                    break;
        
                    case 1:
                    ticks++;
                    
                    if(m_drivetrain.target(0) && ticks > 15){
                        storedAngle = m_drivetrain.getAngle();
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        ticks = 0;
                    }
                        
                    break;
        
                    case 2:
        
                    m_elevator.place();
                    ticks++;
                    if(ticks > 25){
                        m_autoStep = 5;
                        m_drivetrain.resetEncoders();
                        m_elevator.retract();
                    }
        
                    break;

                    case 5:

                    if(m_drivetrain.getEncoder() > -20 ){
                        driveForward(-.5, 0, -80,-100);
                    }else{
                        if(driveForward(-.5, 65, -115,-130)){
                            m_autoStep++;
                            ticks = 0;
                        }
                    }
                    
                    break;

                    case 6:

                    ticks++;

                    m_drivetrain.turnTo(170);
                    m_elevator.extend();
                    if(m_drivetrain.getAngle() > 165 && ticks > 60){
                        m_autoStep++;
                        ticks = 0;
                        ticks2 = 0;
                    }

                    break;

                    case 7:

                    ticks++;
                    if(m_drivetrain.target(0) && ticks > 60){
                        ticks2++;
                        m_drivetrain.tank(0,0);
                        m_drivetrain.resetEncoders();
                        m_elevator.grab();

                        if(ticks2 > 10){
                            m_autoStep++;
                        }
                    }

                    break;

                    case 8:

                    if(driveForward(-.5, 230, -100,-110)){
                        m_autoStep++;
                        ticks = 0;
                    }

                    break;

                    case 9:

                    m_drivetrain.turnTo(320);
                    m_elevator.setLevel(LiftLevels.HATCH_HIGH);
                    m_elevator.extend();
                    if(m_drivetrain.getAngle() > 310){
                        m_autoStep++;
                        m_drivetrain.tank(0,0);
                        ticks = 0;
                    }

                    break;

                    case 10:

                    ticks++;
                    if(m_drivetrain.target(0) && ticks > 60){
                        m_autoStep++;
                        m_drivetrain.tank(0,0);
                        m_elevator.place();
                        ticks = 0;
                    }

                    break;

                    case 11:
                    m_drivetrain.tank(0,0);
                    ticks++;
                    if(ticks > 20){
                        m_elevator.retract();
                    }

                    break;
                }

            break;

            case ROCKET_LEFT:

            break;

            case CARGO_RIGHT:

            switch(m_autoStep){

                case 0:

                m_elevator.setLevel(LiftLevels.HATCH_LOW);
                double speed = 0.5;
                if(m_drivetrain.getEncoder() > 40){
                    speed = 0.70;
                }

                if(driveForward(speed, -15, 135,165)){
                    m_autoStep++;     
                    m_elevator.grab();
                    m_elevator.extend();
                }

                break;

                case 1:

                m_drivetrain.turnTo(85);
                if(m_drivetrain.getAngle() > 82){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    ticks = 0;
                    ticks2 = 0;
                }

                break;

                case 2:

                ticks++;
                if(m_drivetrain.target(90) && ticks > 15){
                    m_elevator.place();
                    if(ticks2 > 5){
                        m_autoStep++;
                    }
                    ticks2++;
                    
                }

                break;

                case 3:

                m_drivetrain.tank(0,0);
                

                break;
            }

            break;

            case CARGO_LEFT:

                switch(m_autoStep){

                    case 0:

                    m_elevator.setLevel(LiftLevels.HATCH_LOW);

                    if(driveForward(0.5, 10, 176,192)){
                        m_autoStep++;     
                        m_elevator.grab();
                    }

                    break;

                    case 1:

                    m_drivetrain.turnTo(-90);
                    if(m_drivetrain.getAngle() > -87){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        ticks = 0;
                    }

                    break;

                    case 2:

                    ticks++;
                    if(m_drivetrain.target(-90) && ticks > 15){
                        
                        m_autoStep++;
                    }

                    break;

                    case 3:

                    m_drivetrain.tank(0,0);

                    break;
                }
            break;
            
        }

        
        
    }

    double maxDeviation = 2;

    MiniPID m_driveLoop = new MiniPID(1,0,0);

    public boolean driveForward(double speed, double angle, double point1, double point2){

        double turn = 0;
        if(m_drivetrain.getAngle() > angle+maxDeviation){

            if(angle < m_drivetrain.getAngle()-5){
                turn = 0.35;
            }else{
                turn = 0.25;
            }
        }

        if(m_drivetrain.getAngle() < angle-maxDeviation){
            if(angle > m_drivetrain.getAngle()+5){
                turn = -0.35;
            }else{
                turn = -0.25;
            }
        }
        System.out.println(m_drivetrain.getEncoder());

        if(speed > 0){
            if(m_drivetrain.getEncoder() < point1){
                m_drivetrain.arcadeDrive(-speed, turn);
                return false;
            }else if(m_drivetrain.getEncoder() < point2){
                m_drivetrain.arcadeDrive(-speed*0.5, turn);
                return false;
            }
        }else{
            if(m_drivetrain.getEncoder() > point1){
                m_drivetrain.arcadeDrive(-speed, turn);
                return false;
            }else if(m_drivetrain.getEncoder() > point2){
                m_drivetrain.arcadeDrive(-speed*0.5, turn);
                return false;
            }
        }
        

        m_drivetrain.arcadeDrive(0,0);

        return true;
    }

    public void teleopInit() {
        //m_climber.raiseFront();
    }

    int pressState = 0;
    int grabState = 0;
    public void teleopPeriodic() {     
        
        m_drivetrain.arcadeDrive(m_joystick.getRawAxis(1), m_joystick.getRawAxis(2));

        // Switches to Place Panel mode and cancels any passthrough functions
        if(m_joystick.getRawButton(5)){
            m_robotState.setState(State.PLACE_PANEL);
        }

        // Switches to Grab Cargo mode
        if(m_joystick.getRawButton(3) && (m_robotState.state() == State.PLACE_PANEL || m_robotState.state() == State.GRAB_CARGO_FEEDER)){
            m_robotState.setState(State.GRAB_CARGO);
        }

        // Switches to Grab Cargo from feeder station mode
        if(m_joystick.getRawButton(4) && (m_robotState.state() == State.PLACE_PANEL || m_robotState.state() == State.GRAB_CARGO)){
            m_robotState.setState(State.GRAB_CARGO_FEEDER);
        }
  
        if(m_robotState.state() == State.PLACE_PANEL){

            // ELEVATOR CONTROLS
            if(m_joystick.getRawButton(7))
                m_elevator.setLevel(LiftLevels.HATCH_HIGH);
            if(m_joystick.getRawButton(9))
                m_elevator.setLevel(LiftLevels.HATCH_MEDIUM);
            if(m_joystick.getRawButton(11))
                m_elevator.setLevel(LiftLevels.HATCH_LOW);

            // Controls the state of the passive grabber
            SmartDashboard.putNumber("GrabState", grabState);
            switch(grabState){
                case 0:
                    grabState = 0;
                    if(m_joystick.getRawButton(2))
                        grabState = 1;
                    break;

                case 1:
                    
                    grabState = 1;

                    if(!m_joystick.getRawButton(2)){
                        grabState = 2;
                        m_elevator.grab();
                    }
                    break;

                case 2:

                    grabState = 2;
                    
                    if(m_joystick.getRawButton(2)){
                        grabState = 3;
                    }
                        
                    break;

                case 3:

                    grabState = 3;
                    
                    if(!m_joystick.getRawButton(2)){
                        grabState = 0;
                        m_elevator.place();
                    }
                    break;
            }      
        }

        if(m_robotState.state() == State.PLACE_CARGO){     
            // ELEVATOR HEIGHTS
            if(m_joystick.getRawButton(7))
                m_elevator.setLevel(LiftLevels.PORT_HIGH);
            if(m_joystick.getRawButton(12))
                m_elevator.setLevel(LiftLevels.PORT_CARGO_SHIP);
            if(m_joystick.getRawButton(9))
                m_elevator.setLevel(LiftLevels.PORT_MEDIUM);
            if(m_joystick.getRawButton(11))
                m_elevator.setLevel(LiftLevels.PORT_LOW);       
        }

        // GRANULAR ELEVATOR CONTROLS
        if(m_joystick.getPOV(0) == 0){
            m_elevator.granular(1);
        }
        if(m_joystick.getPOV(0) == 180){
            m_elevator.granular(-1);
        }

        // PLACES CARGO
        if(m_joystick.getRawButton(1)){
            m_cargoSystem.shoot(0.5);
        }

        // Vision Processing
        if(m_joystick.getRawButton(12)){ 
            m_drivetrain.target(-30);
            return;
        }

        // Reverse Passthrough
        if(m_nav.getRawButton(1)){
            m_robotState.setState(State.PLACE_PANEL);
            m_cargoSystem.runPassthrough(-1);
        }

        controlMechanisms();
    }

    public void disabledInit(){
        m_drivetrain.tank(0,0);
        m_elevator.reset();
        
    }

    /** Utilizes the Teleop panel grabber switch statement in Auto to extend and retract panel grabber */
    public void extend(){
        m_pokeState = 2;
    }
    public void retract(){
        m_pokeState = 0;
    }

    public void testPeriodic(){
        //m_climber.lowerBack();
    }

    /** Instrum Code */
    public void robotPeriodic(){
        SmartDashboard.putNumber("Gyro", m_drivetrain.getAngle());
        SmartDashboard.putNumber("Elevator Position", m_elevator.getRaw());
        SmartDashboard.putBoolean("passthrough sensor", m_cargoSystem.getPassthroughSensor());
        SmartDashboard.putBoolean("back shooter sensor", m_cargoSystem.getBackShooterSensor());
        SmartDashboard.putBoolean("front shooter sensor", m_cargoSystem.getFrontShooterSensor()); 
        SmartDashboard.putNumber("drivetrain position", m_drivetrain.getEncoder());
        
        m_elevator.run();
        m_cargoSystem.run();
        m_climber.run();   
    }

    public void controlMechanisms(){

        // Controls the grabber
        switch(m_pokeState){
            case 0:
            m_pokeState = 0;
                m_elevator.retract();
                if(m_joystick.getRawButton(6))
                    m_pokeState = 1;
                break;

            case 1:
                
                m_pokeState = 1;

                if(!m_joystick.getRawButton(6)){
                    m_pokeState = 2;
                    
                }
                break;

            case 2:

                m_pokeState = 2;
                m_elevator.extend();
                if(m_joystick.getRawButton(6)){
                    m_pokeState = 3;
                }
                    
                break;

            case 3:

                m_pokeState = 3;
                
                if(!m_joystick.getRawButton(6)){
                    m_pokeState = 0;
                    
                }
                break;
        }

        if(m_robotState.state() == State.PLACE_PANEL){
            // Controls the climbing system
            switch(m_cylinderState){
                case 0:

                    m_cylinderState = 0;
                    
                    if(m_joystick.getRawButton(8))
                        m_cylinderState = 1;
                    break;

                case 1:
                    
                    m_cylinderState = 1;

                    if(!m_joystick.getRawButton(8)){
                        m_climber.lowerFront();
                        m_cylinderState = 2;
                        
                    }
                    break;

                case 2:

                    m_cylinderState = 2;
                    
                   
                    if(m_joystick.getRawButton(8)){
                        m_cylinderState = 3;
                    }
                        
                    break;

                case 3:

                    m_cylinderState = 3;
                    
                    if(!m_joystick.getRawButton(8)){
                        m_climber.raiseFront();
                        m_cylinderState = 0;
                        
                    }
                    break;
            }
            switch(m_pegState){
                case 0:
                    m_pegState = 0;
                    
                    if(m_joystick.getRawButton(10))
                        m_pegState = 1;
                    break;

                case 1:
                    
                    m_pegState = 1;

                    if(!m_joystick.getRawButton(10)){
                        System.out.println("up");
                        m_climber.lowerBack();
                        m_pegState = 2;
                        
                    }
                    break;

                case 2:

                    m_pegState = 2;
                    
                    if(m_joystick.getRawButton(10)){
                        m_pegState = 3;
                    }
                        
                    break;

                case 3:

                    m_pegState = 3;
                    
                    if(!m_joystick.getRawButton(10)){
                        m_climber.raiseBack();
                        m_pegState = 0;
                        
                    }
                    break;
            }
        }else{
            //m_climber.raiseFront();
            //m_climber.raiseBack();
        }
    }

    

    enum Auto{
        ROCKET_RIGHT, ROCKET_LEFT, CARGO_RIGHT, CARGO_LEFT, MANUAL;
    }
}
