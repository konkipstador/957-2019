/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    Compressor m_compressor = new Compressor(6);

    boolean m_autoState = false;

    int m_autoPhase = 0;
    int m_frontClimberState = 0;
    int m_backClimberState = 0;
    int m_pokeState = 0;
    int m_climberStandState = 0;
    int m_totalClimberState = 0;

    Auto m_autoMode = Auto.MANUAL;

    int m_countdown = 0;

    PowerDistributionPanel m_pdp = new PowerDistributionPanel(11);

    SendableChooser<Auto> m_autoSelector = new SendableChooser<Auto>();

    int lastSelected = 2;

    // Driving Cotnrols
    int k_shoot = 1;    // Shoots Cargo
    int k_actuatePanel = 2; // Moves the elevator up or down slightly to grab and release panels
    int k_toGrabMode = 3;   // Sets the robot to a standard cargo grabbing mode
    int k_toFeederGrabMode = 4; // Sets the robot to grab from the feeder station
    int k_toPanelMode = 5;  // If the robot is in a passthrough or cargo grabbing mode, resets the robot back to panel grabbing
    int k_poke = 6;     // Actuates the panel grabber piston
    int k_elevatorHigh = 7;
    int k_elevatorCargoShip = 8;
    int k_elevatorMed = 9;
    int k_reversePassthrough = 10;  // Reverses the passthrough and moves back to a panel grabbing mode
    int k_elevatorLow = 11;
    int k_visionTarget = 12;     // Perform vision targeting on a target
    
    int k_driveAxis = 1;
    int k_turnAxis = 2;   

    // Navigator Controls (Xbox Controller)
    int k_actuateFrontCylinders = 4;
    int k_actuateStand = 2;
    int k_actuateAll = 3;
    int k_actuateBackCylinders = 1;

    ComplexWidget m_selector;
    
    public void robotInit() {      
        m_robotState.setState(State.PLACE_PANEL);     
        m_drivetrain.resetNavX();

        // Configure Auto boolean fields
        SmartDashboard.putBoolean("Rocket Right", false);
        SmartDashboard.putBoolean("Rocket Left", false);
        SmartDashboard.putBoolean("Cargo Right", false);
        SmartDashboard.putBoolean("Cargo Left", false);
        SmartDashboard.putBoolean("Front Left Cargo", false);
        SmartDashboard.putBoolean("Manual",true);

        m_climber.raiseBack();
        m_climber.raiseFront();
        m_climber.raiseStand();
        m_elevator.retract();
    }

    public void autonomousInit() {
        m_drivetrain.resetEncoders();
        m_drivetrain.resetNavX();
        m_autoPhase = 0;
        m_autoStep = 0;
        m_countdown = 0; 
    }

    int m_autoStep = 0;
    int ticks = 0;
    int ticks2 = 0;
    double storedAngle = 0;

    public void autonomousPeriodic() {

        if(m_joystick.getRawButton(1)){
            m_autoMode = Auto.MANUAL;
            lastSelected = 2;
            SmartDashboard.putBoolean("Rocket Right", false);
            SmartDashboard.putBoolean("Rocket Left", false);
            SmartDashboard.putBoolean("Cargo Right", false);
            SmartDashboard.putBoolean("Cargo Left", false);
            SmartDashboard.putBoolean("Front Left Cargo", false);
            SmartDashboard.putBoolean("Manual", false);
        }
        
        switch(m_autoMode){
            
            case ROCKET_RIGHT:

                switch(m_autoStep){
                    case 0:
                    m_elevator.retract();
                    if(driveForward(0.7, -2.5, 55,65)){
                        m_autoStep++;
                        m_elevator.setLevel(LiftLevels.HATCH_LOW);
                        m_elevator.grab();
                        m_elevator.extend();
                        ticks = 0;
                        m_vision.enableTracking();
                    }
        
                    break;
        
                    case 1:
                    ticks++;
                    
                    if(m_drivetrain.target(0) && ticks > 15){
                        storedAngle = m_drivetrain.getAngle();
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        ticks = 0;
                        m_vision.disableTracking();
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

                    if(m_drivetrain.getEncoder() > -10 ){
                        driveForward(-.65, 0, -80,-100);
                    }else{
                        if(driveForward(-.7, 60, -110,-115)){
                            m_autoStep++;
                            ticks = 0;
                        }
                    }
                    
                    break;

                    case 6:

                    ticks++;

                    m_drivetrain.turnTo(190);
                    m_elevator.extend();
                    if(m_drivetrain.getAngle() > 180 && ticks > 60){
                        m_autoStep++;
                        ticks = 0;
                        ticks2 = 0;
                        m_vision.enableTracking();
                    }

                    break;

                    case 7:

                    ticks++;
                    if(m_drivetrain.target(0) && ticks > 60){
                        ticks2++;
                        m_drivetrain.tank(0,0);
                        m_drivetrain.resetEncoders();
                        m_elevator.grab();

                        if(ticks2 > 15){
                            m_autoStep++;
                            m_vision.disableTracking();
                        }
                    }

                    break;

                    case 8:

                    if(driveForward(-.7, 230, -90,-110)){
                        m_autoStep++;
                        ticks = 0;
                    }

                    break;

                    case 9:

                    m_drivetrain.turnTo(310);
                    m_elevator.setLevel(LiftLevels.HATCH_HIGH);
                    m_elevator.extend();
                    if(m_drivetrain.getAngle() > 310){
                        m_autoStep++;
                        m_drivetrain.tank(0,0);
                        ticks = 0;
                        m_vision.enableTracking();
                        m_drivetrain.refreshTargetAngle();
                    }

                    break;

                    case 10:

                    ticks++;
                    if(m_drivetrain.targetAngle(360) && ticks > 100){
                        m_autoStep++;
                        m_drivetrain.tank(0,0);
                        m_elevator.place();
                        ticks = 0;
                        m_vision.disableTracking();
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

            switch(m_autoStep){
                
                case 0:
                double sspeed = -.9;
                if(m_drivetrain.getEncoder() > -170){
                    sspeed = -.75;
                }

                if(driveForward(sspeed, 20, -180, -200)){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                }

                break;

                case 1:

                

                if(m_drivetrain.getAngle() < -25){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    m_elevator.extend();
                    m_elevator.grab();
                    ticks = 0;
                    ticks2 = 0;
                    m_vision.enableTracking();
                }else{
                    m_drivetrain.autocade(0,.12);
                }

                break;

                case 2:

                ticks++;
                if(ticks > 75){
                    
                    m_drivetrain.tank(0,0);
                    m_elevator.place();
                    ticks2++;

                    if(ticks2 > 15){
                        m_autoStep++;
                        m_vision.disableTracking();
                        ticks = 0;
                        ticks2 = 0;
                        m_drivetrain.resetEncoders();
                    }
                }else{
                    m_drivetrain.targetAngle(30);
                }

                break;

                case 3:

                ticks++;

                if(ticks > 25){
                    if(driveForward(-0.75,-30, -10,-20)){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        m_elevator.retract();
                    };
                }

                break;

                case 4:

                if(m_drivetrain.getAngle() > 0){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    m_drivetrain.resetEncoders();
                }else{
                    m_drivetrain.autocade(0,-.25);
                }

                break;

                case 5:

                double angle = 5;
                double speed = .8;
                if(m_drivetrain.getEncoder() > 95){
                    angle = -30;
                     speed = .75;
                }
                if(m_drivetrain.getEncoder() > 200){
                    angle = 0;
                    speed = .65;
                }

                if(driveForward(speed,angle, 230,230)){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    ticks = 0;
                    m_elevator.extend();
                    m_vision.enableTracking();
                };

                break;

                case 6:

                ticks++;
                if(ticks > 75){
                    ticks2++;
                    m_drivetrain.tank(0,0);
                    if(ticks2 > 25){
                        m_autoStep++;
                    }
                    
                    m_elevator.grab();
                    m_vision.disableTracking();
                    m_drivetrain.resetEncoders();
                }else{
                    m_drivetrain.targetAngle(10);
                }

                break;

                case 7:
                    m_elevator.retract();
                    double speed2 = -.65;
                    if( m_drivetrain.getEncoder() > 100){
                        speed2 = -0.3;
                    }
                    if(driveForward(speed2,-10, -120,-140)){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        ticks = 0;
                        
                    };

                break;

                case 8:

                    if(m_drivetrain.getAngle() > -130){
                        m_drivetrain.arcadeDrive(.0,.15);
                    }else{
                        m_drivetrain.tank(0,0);
                    }

                    
                break;

            }



            break;

            case CARGO_RIGHT:

                switch(m_autoStep){

                    case 0:

                    m_elevator.setLevel(LiftLevels.HATCH_LOW);
                    double speed = 0.5;
                    if(m_drivetrain.getEncoder() > 40){
                        speed = 0.70;
                    }

                    if(driveForward(speed, -17, 135,156)){
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
                        m_vision.enableTracking();
                    }

                    break;

                    case 2:

                    

                    if(m_drivetrain.faceTarget()){
                        m_autoStep++;
                    }
                    

                    break;

                    case 3:
                    ticks++;
                    if(m_drivetrain.target(90) && ticks > 50){
                        m_elevator.place();
                        if(ticks2 > 25){
                            m_autoStep++;
                            m_elevator.retract();
                            m_drivetrain.resetEncoders();
                            m_vision.disableTracking();
                        }
                        ticks2++;
                        
                    }

                    break;

                    case 4:

                    if(driveForward(-.5, 90, -12, -18)){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                    }
                    

                    break;

                    case 5:

                    m_drivetrain.turnTo(130);
                    ticks++;
                    if(m_drivetrain.getAngle() > 130){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        ticks = 0;
                        ticks2 = 0;
                        m_drivetrain.resetEncoders();
                    }


                    break;

                    case 6:

                    if(driveForward(0.6, 190, 175, 205)){
                        m_elevator.extend();
                        m_autoStep++;
                        ticks = 0;
                        
                    }
                    
                    break;

                    case 7:

                    m_drivetrain.turnTo(180);
                    ticks++;
                    if(m_drivetrain.getAngle() < 185){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                        ticks = 0;
                        ticks2 = 0;
                        m_drivetrain.resetEncoders();
                        m_vision.enableTracking();
                    }

                    break;

                    case 8:
                    
                        ticks++;
                        if(m_drivetrain.target(0) && ticks > 20){
                            m_elevator.grab();
                            ticks2++;
                            if(ticks2 > 20){
                                m_drivetrain.tank(0,0);
                                m_autoStep++;
                                m_drivetrain.resetEncoders();
                                m_vision.disableTracking();
                            }
                            
                        }
                        
                    break;

                    case 9:

                    if(driveForward(-.7, 200, -90,-110)){
                        m_autoStep++;
                        ticks = 0;
                        m_drivetrain.arcadeDrive(0,0);
                        m_drivetrain.resetEncoders();
                    }

                    break;

                    case 10:

                    m_drivetrain.arcadeDrive(0,0);

                    break;
                }

            break;

            case CARGO_LEFT:

            switch(m_autoStep){

                case 0:

                m_elevator.setLevel(LiftLevels.HATCH_LOW);
                double speed = 0.5;
                double angle = 0;
                if(m_drivetrain.getEncoder() < -80){
                    speed = 0.70;
                    angle = 17;
                }

                if(driveForward(-speed, angle, -135,-159)){
                    m_autoStep++;     
                    m_elevator.grab();
                    m_elevator.extend();
                }

                break;

                case 1:

                m_drivetrain.turnTo(85);
                if(m_drivetrain.getAngle() > 77){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    m_autoStep++;
                    ticks = 0;
                    ticks2 = 0;
                    m_vision.enableTracking();
                }

                break;

                case 2:

                

                if(m_drivetrain.faceTarget()){
                    m_autoStep++;
                }
                

                break;

                case 3:
                ticks++;
                if(m_drivetrain.target(90) && ticks > 50){

                    m_drivetrain.arcadeDrive(0, 0);
                    m_autoStep = 20;
                    m_drivetrain.resetEncoders();      
                    ticks2 =0;
                    
                }

                break;


                case 20:

                if(m_drivetrain.getLeftEncoder() < -0.1){
                    m_elevator.place();
                    ticks2++;
                    m_drivetrain.arcadeDrive(0,0);
                }else{
                    m_drivetrain.autocade(0.05,0);
                }
                
                if(ticks2 > 25){
                    
                    m_elevator.retract();
                    m_autoStep = 4;
                    m_vision.disableTracking();
                }

                break;

                case 4:

                if(driveForward(-.5, 90, -12, -18)){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                }
                

                break;

                case 5:

                m_drivetrain.turnTo(0);
                ticks++;
                if(m_drivetrain.getAngle() < 10){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    ticks = 0;
                    ticks2 = 0;
                    m_drivetrain.resetEncoders();
                }


                break;

                case 6:

                if(driveForward(0.6, -10, 175, 205)){
                    m_elevator.extend();
                    m_autoStep++;
                    ticks = 0;
                    
                }
                
                break;

                case 7:

                m_drivetrain.turnTo(0);
                ticks++;
                if(m_drivetrain.getAngle() > -5){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    ticks = 0;
                    ticks2 = 0;
                    m_drivetrain.resetEncoders();
                    m_vision.enableTracking();
                }

                break;

                case 8:
                
                    ticks++;
                    if(m_drivetrain.target(0) && ticks > 20){
                        m_elevator.grab();
                        ticks2++;
                        if(ticks2 > 20){
                            m_drivetrain.tank(0,0);
                            m_autoStep++;
                            m_drivetrain.resetEncoders();
                            m_vision.disableTracking();
                        }
                        
                    }
                    
                break;

                case 9:

                if(driveForward(-.7, -20, -90,-110)){
                    m_autoStep++;
                    ticks = 0;
                    m_drivetrain.arcadeDrive(0,0);
                }

                break;

                case 10:

                m_drivetrain.arcadeDrive(0,0);

                break;
            }
            break;

            case FRONT_LEFT_CARGO:

                switch(m_autoStep){

                    case 0:

                    if(m_drivetrain.getEncoder() > 50){
                        m_elevator.extend();
                        m_elevator.grab();
                    }

                    if(driveForward(0.6, 0, 70, 80)){
                        m_drivetrain.tank(0,0);
                        m_autoStep++;
                    }
                    break;
                }

            

            break;

            case MANUAL:

                teleopPeriodic();

            break;
            
        }
        
        m_countdown++;
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
        m_vision.disableTracking();
    }

    int pressState = 0;
    int grabState = 0;
    public void teleopPeriodic() {     
        
        if(m_joystick.getRawAxis(3) < 0){
            m_drivetrain.arcadeDrive(m_joystick.getRawAxis(k_driveAxis), m_joystick.getRawAxis(k_turnAxis));
        }else{
            m_drivetrain.arcadeDrive(m_joystick.getRawAxis(k_driveAxis)/2, m_joystick.getRawAxis(k_turnAxis)/2);
        }
        

        // Switches to Place Panel mode and cancels any passthrough functions
        if(m_joystick.getRawButton(k_toPanelMode)){
            m_robotState.setState(State.PLACE_PANEL);
        }

        // Switches to Grab Cargo mode
        if(m_joystick.getRawButton(k_toGrabMode) && (m_robotState.state() == State.PLACE_PANEL || m_robotState.state() == State.GRAB_CARGO_FEEDER) && m_cargoSystem.getFrontShooterSensor()){
            m_robotState.setState(State.GRAB_CARGO);
        }

        // Switches to Grab Cargo from feeder station mode
        if(m_joystick.getRawButton(k_toFeederGrabMode) && (m_robotState.state() == State.PLACE_PANEL || m_robotState.state() == State.GRAB_CARGO) && m_cargoSystem.getFrontShooterSensor()){
            m_robotState.setState(State.GRAB_CARGO_FEEDER);
        }

        if(m_robotState.state() != State.PLACE_PANEL){
            grabState = 0;
            m_elevator.place();
        }

        if(!m_cargoSystem.getFrontShooterSensor()){
            // ELEVATOR HEIGHTS
            if(m_joystick.getRawButton(k_elevatorHigh))
                m_elevator.setLevel(LiftLevels.PORT_HIGH);
            if(m_joystick.getRawButton(k_elevatorCargoShip))
                m_elevator.setLevel(LiftLevels.PORT_CARGO_SHIP);
            if(m_joystick.getRawButton(k_elevatorMed))
                m_elevator.setLevel(LiftLevels.PORT_MEDIUM);
            if(m_joystick.getRawButton(k_elevatorLow))
                m_elevator.setLevel(LiftLevels.PORT_LOW);       
        }else{

            // ELEVATOR CONTROLS
            if(m_joystick.getRawButton(k_elevatorHigh))
                m_elevator.setLevel(LiftLevels.HATCH_HIGH);
            if(m_joystick.getRawButton(k_elevatorMed))
                m_elevator.setLevel(LiftLevels.HATCH_MEDIUM);
            if(m_joystick.getRawButton(k_elevatorLow))
                m_elevator.setLevel(LiftLevels.HATCH_LOW);
            if(m_joystick.getRawButton(k_elevatorCargoShip))
                m_elevator.setLevel(LiftLevels.PORT_CARGO_SHIP);

            // Controls the state of the passive grabber
            switch(grabState){
                case 0:
                    grabState = 0;
                    if(m_joystick.getRawButton(k_actuatePanel))
                        grabState = 1;
                    break;

                case 1:
                    
                    grabState = 1;

                    if(!m_joystick.getRawButton(k_actuatePanel)){
                        grabState = 2;
                        m_elevator.grab();
                    }
                    break;

                case 2:

                    grabState = 2;
                    
                    if(m_joystick.getRawButton(k_actuatePanel)){
                        grabState = 3;
                    }
                        
                    break;

                case 3:

                    grabState = 3;
                    
                    if(!m_joystick.getRawButton(k_actuatePanel)){
                        grabState = 0;
                        m_elevator.place();
                    }
                    break;
            }      
        }

        

        // GRANULAR ELEVATOR CONTROLS
        if(m_joystick.getPOV(0) == 0){
            m_elevator.granular(1);
        }
        if(m_joystick.getPOV(0) == 180){
            m_elevator.granular(-1);
        }

        // PLACES CARGO
        if(m_joystick.getRawButton(k_shoot)){
            m_cargoSystem.shoot(0.5);
        }

        // Vision Processing
        if(m_joystick.getRawButton(k_visionTarget)){ 
            m_vision.enableTracking();
            m_drivetrain.target(-30);
            return;
        }else{
            m_vision.disableTracking();
        }

        // Reverse Passthrough
        if(m_joystick.getRawButton(k_reversePassthrough)){
            m_robotState.setState(State.PLACE_PANEL);
            m_cargoSystem.runPassthrough(-1);
        }
        
        // Controls the grabber
        switch(m_pokeState){
            case 0:
            m_pokeState = 0;
                
                if(m_joystick.getRawButton(k_poke))
                    m_pokeState = 1;
                break;

            case 1:
                
                m_pokeState = 1;

                if(!m_joystick.getRawButton(k_poke)){
                    m_pokeState = 2;
                    m_elevator.extend();
                    
                }
                break;

            case 2:

                m_pokeState = 2;
                
                if(m_joystick.getRawButton(k_poke)){
                    m_pokeState = 3;
                }
                    
                break;

            case 3:

                m_pokeState = 3;
                
                if(!m_joystick.getRawButton(k_poke)){
                    m_elevator.retract();
                    m_pokeState = 0;
                    
                }
                break;
        }

        if(m_nav.getRawButton(8)){
            m_frontClimberState = 0;
            m_backClimberState = 0;
            m_climberStandState = 0;
            m_totalClimberState = 0;

            m_climber.raiseBack();
            m_climber.raiseStand();
            m_climber.raiseFront();
        }

        // CLIMBER CONTROLS
        switch(m_frontClimberState){
            case 0:

                m_frontClimberState = 0;
                
                if(m_nav.getRawButton(k_actuateFrontCylinders))
                    m_frontClimberState = 1;
                break;

            case 1:
                
                m_frontClimberState = 1;

                if(!m_nav.getRawButton(k_actuateFrontCylinders)){
                    m_climber.lowerFront();
                    m_frontClimberState = 2;
                    
                }
                break;

            case 2:

                m_frontClimberState = 2;
                
                
                if(m_nav.getRawButton(k_actuateFrontCylinders)){
                    m_frontClimberState = 3;
                }
                    
                break;

            case 3:

                m_frontClimberState = 3;
                
                if(!m_nav.getRawButton(k_actuateFrontCylinders)){
                    m_climber.raiseFront();
                    m_frontClimberState = 0;
                    
                }
                break;
        }

        switch(m_backClimberState){
            case 0:
                m_backClimberState = 0;
                
                if(m_nav.getRawButton(k_actuateBackCylinders))
                    m_backClimberState = 1;
                break;

            case 1:
                
                m_backClimberState = 1;

                if(!m_nav.getRawButton(k_actuateBackCylinders)){
                    m_climber.lowerBack();
                    m_backClimberState = 2;
                    
                }
                break;

            case 2:

                m_backClimberState = 2;
                
                if(m_nav.getRawButton(k_actuateBackCylinders)){
                    m_backClimberState = 3;
                }
                    
                break;

            case 3:

                m_backClimberState = 3;
                
                if(!m_nav.getRawButton(k_actuateBackCylinders)){
                    m_climber.raiseBack();
                    m_backClimberState = 0;
                    
                }
                break;
        }

        switch(m_climberStandState){
            case 0:
                m_climberStandState = 0;
                
                if(m_nav.getRawButton(k_actuateStand))
                    m_climberStandState = 1;
                break;

            case 1:
                
                m_climberStandState = 1;

                if(!m_nav.getRawButton(k_actuateStand)){
                    m_climber.lowerStand();
                    m_climberStandState = 2;
                    
                }
                break;

            case 2:

                m_climberStandState = 2;
                
                if(m_nav.getRawButton(k_actuateStand)){
                    m_climberStandState = 3;
                }
                    
                break;

            case 3:

                m_climberStandState = 3;
                
                if(!m_nav.getRawButton(k_actuateStand)){
                    m_climber.raiseStand();
                    m_climberStandState = 0;
                    
                }
                break;
        }

        
        switch(m_totalClimberState){
            case 0:
            m_totalClimberState = 0;
                
                if(m_nav.getRawButton(k_actuateAll))
                    m_totalClimberState = 1;
                break;

            case 1:
                
                m_totalClimberState = 1;

                if(!m_nav.getRawButton(k_actuateAll)){
                    m_climber.lowerFront();
                    m_totalClimberState = 2;
                    
                }
                break;

            case 2:

                m_totalClimberState = 2;
                
                if(m_nav.getRawButton(k_actuateAll)){
                    m_totalClimberState = 3;
                }
                    
                break;

            case 3:

                m_totalClimberState = 3;
                
                if(!m_nav.getRawButton(k_actuateAll)){
                    m_climber.raiseFront();
                    m_totalClimberState = 4;
                    
                }
                break;
            case 4:
                m_totalClimberState = 4;
                
                if(m_nav.getRawButton(k_actuateAll))
                    m_totalClimberState = 5;
                break;

            case 5:
                
                m_totalClimberState = 5;

                if(!m_nav.getRawButton(k_actuateAll)){
                    m_climber.lowerBack();
                    m_totalClimberState = 6;
                    
                }
                break;

            case 6:

                m_totalClimberState = 6;
                
                if(m_nav.getRawButton(k_actuateAll)){
                    m_totalClimberState = 7;
                    
                }
                    
                break;

            case 7:

                m_totalClimberState = 7;
                
                if(!m_nav.getRawButton(k_actuateAll)){
                    m_climber.lowerStand();
                    m_totalClimberState = 8;
                    
                }
                break;
            case 8:
                m_totalClimberState = 8;
                
                if(m_nav.getRawButton(k_actuateAll))
                    m_totalClimberState = 9;
                break;

            case 9:
                
                m_totalClimberState = 9;

                if(!m_nav.getRawButton(k_actuateAll)){
                    m_climber.raiseBack();
                    m_totalClimberState = 10;
                    
                }
                break;

            case 10:

                m_totalClimberState = 10;
                
                if(m_nav.getRawButton(k_actuateAll)){
                    m_totalClimberState = 0;
                    
                }
                    
                break;

        }
    
        m_countdown++;
    }

    public void disabledInit(){
        m_drivetrain.tank(0,0);
        m_elevator.reset();
        
    }

    public void testPeriodic(){
    }

    /** Instrum Code */
    public void robotPeriodic(){     
        m_elevator.run();
        m_cargoSystem.run();
        updateDashboard();
    }

    enum Auto{
        ROCKET_RIGHT, ROCKET_LEFT, CARGO_RIGHT, CARGO_LEFT, MANUAL, FRONT_LEFT_CARGO;
    }

    //DigitalInput m_panelSensor = new DigitalInput(3);

    public void updateDashboard(){
        
        // Update Gyro Position
        SmartDashboard.putNumber("Gyro Angle", m_drivetrain.getAngle());
        //SmartDashboard.putNumber("Panel Sensor", m_panelSensor.getVoltage());

        // Update Elevator Position
        SmartDashboard.putNumber("Raw Elevator Position", m_elevator.getRaw());
        SmartDashboard.putString("Last Set Elevator Position", m_elevator.getLevel().identifier());

        // Update Sensors
        SmartDashboard.putBoolean("Passthrough Sensor", !m_cargoSystem.getPassthroughSensor());
        SmartDashboard.putBoolean("Back Shooter Sensor", !m_cargoSystem.getBackShooterSensor());
        SmartDashboard.putBoolean("Front Shooter Sensor", !m_cargoSystem.getFrontShooterSensor()); 
        SmartDashboard.putNumber("Drivetrain Encoder Position", m_drivetrain.getEncoder());
        SmartDashboard.putNumber("Vision Target Position", m_vision.getTargetLocation());

        // Current Draw Data
        SmartDashboard.putNumber("System-Wide Current Draw", m_pdp.getTotalCurrent());
        SmartDashboard.putNumber("Drivetrain Current Draw", m_drivetrain.getCurrent());
        SmartDashboard.putNumber("Elevator Current Draw", m_elevator.getCurrent());
        SmartDashboard.putNumber("Passthrough Current Draw", m_cargoSystem.getCurrent());
        SmartDashboard.putNumber("Compressor Current Draw", m_compressor.getCompressorCurrent());

        // Other Data
        SmartDashboard.putNumber("Match Countdown", m_countdown/50);
        SmartDashboard.putString("Selected Auto Mode", m_autoMode.toString());
        SmartDashboard.putBoolean("Pneumatics Charging", !m_compressor.getPressureSwitchValue());

        if(SmartDashboard.getBoolean("Rocket Right", false) && lastSelected != 0){
            m_autoMode = Auto.ROCKET_RIGHT;
            lastSelected = 0;
            SmartDashboard.putBoolean("Manual", false);
            SmartDashboard.putBoolean("Cargo Right", false);
            SmartDashboard.putBoolean("Cargo Left", false);
            SmartDashboard.putBoolean("Rocket Left", false);
            SmartDashboard.putBoolean("Front Left Cargo", false);
        }

        if(SmartDashboard.getBoolean("Cargo Right", false) && lastSelected != 1){
            m_autoMode = Auto.CARGO_RIGHT;
            lastSelected = 1;
            SmartDashboard.putBoolean("Manual", false);
            SmartDashboard.putBoolean("Rocket Right", false);
            SmartDashboard.putBoolean("Cargo Left", false);
            SmartDashboard.putBoolean("Rocket Left", false);
            SmartDashboard.putBoolean("Front Left Cargo", false);
        }   

        if(SmartDashboard.getBoolean("Manual", false) && lastSelected != 2){
            m_autoMode = Auto.MANUAL;
            lastSelected = 2;
            SmartDashboard.putBoolean("Rocket Right", false);
            SmartDashboard.putBoolean("Cargo Right", false);
            SmartDashboard.putBoolean("Cargo Left", false);
            SmartDashboard.putBoolean("Rocket Left", false);
            SmartDashboard.putBoolean("Front Left Cargo", false);
        }

        if(SmartDashboard.getBoolean("Cargo Left", false) && lastSelected != 3){
            m_autoMode = Auto.CARGO_LEFT;
            lastSelected = 3;
            SmartDashboard.putBoolean("Manual", false);
            SmartDashboard.putBoolean("Rocket Right", false);
            SmartDashboard.putBoolean("Cargo Right", false);
            SmartDashboard.putBoolean("Rocket Left", false);
            SmartDashboard.putBoolean("Front Left Cargo", false);
        }

        if(SmartDashboard.getBoolean("Rocket Left", false) && lastSelected != 4){
            m_autoMode = Auto.ROCKET_LEFT;
            lastSelected = 4;
            SmartDashboard.putBoolean("Manual", false);
            SmartDashboard.putBoolean("Rocket Right", false);
            SmartDashboard.putBoolean("Cargo Right", false);
            SmartDashboard.putBoolean("Cargo Left", false);
            SmartDashboard.putBoolean("Front Left Cargo", false);
        }
        
        if(SmartDashboard.getBoolean("Front Left Cargo", false) && lastSelected != 5){
            m_autoMode = Auto.FRONT_LEFT_CARGO;
            lastSelected = 5;
            SmartDashboard.putBoolean("Manual", false);
            SmartDashboard.putBoolean("Rocket Right", false);
            SmartDashboard.putBoolean("Cargo Right", false);
            SmartDashboard.putBoolean("Cargo Left", false);
            SmartDashboard.putBoolean("Rocket Left", false);
        }


    }
}