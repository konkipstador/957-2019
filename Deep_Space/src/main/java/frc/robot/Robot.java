/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.libraries.MiniPID;
import frc.libraries.VL53L0X;
import frc.robot.Elevator.LiftLevels;
import frc.robot.RobotState.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

    Drivetrain m_drivetrain = Drivetrain.getInstance();
    Elevator m_elevator = Elevator.getInstance();
    RobotState m_robotState = RobotState.getInstance();
    Vision m_vision = Vision.getInstance();
    CargoSystem m_cargoSystem = CargoSystem.getInstance();
    Joystick m_joystick = new Joystick(0);
    Joystick m_nav = new Joystick(1);

    boolean m_autoState = false;
    VL53L0X sensor1;


    int m_autoPhase = 0;
    
    public void robotInit() {      
        m_robotState.setState(State.PLACE_PANEL);     
        m_drivetrain.resetNavX();  
        try{
            sensor1 = new VL53L0X(0x29);
        }catch(Exception e){
            System.out.println("Sensor failed to initalize.");
        }
       
        
    }

    public void autonomousInit() {
        m_drivetrain.resetEncoders();
        
        m_autoPhase = 0;
        m_autoStep = 0;
    }

    int m_autoStep = 0;
    int ticks = 0;

    public void autonomousPeriodic() {
        //m_pathweaver.runPath(0);
        //m_cargoSystem.drive(m_joystick.getRawAxis(1));

        //m_drivetrain.turnTo(180);

        if(true){
            switch(m_autoStep){
                case 0:
    
                if(driveForward(0.5, 0, 55,65)){
                    m_autoStep++;
                    m_elevator.setLevel(LiftLevels.HATCH_LOW);
                    m_elevator.grab();
                }
    
                break;
    
                case 1:
    
                m_drivetrain.turnTo(-75);
                if(m_drivetrain.getAngle() < -71){
                    m_autoStep++;
                    m_drivetrain.resetEncoders();
                }
                    
                break;
    
                case 2:
    
                if(driveForward(0.5,-75, 32,37)){
                    m_autoStep = 3;
                }
    
                break;
    
                case 3:
                    m_drivetrain.turnTo(-30);
                    if(m_drivetrain.getAngle() > -35){
                        m_autoStep++;
                        ticks = 0;
                    }
                break;
    
                case 4:
    
                ticks++;
    
                if(m_drivetrain.target(-30) && ticks > 20){
                    //m_autoStep++;
                    m_drivetrain.resetEncoders();
                    m_drivetrain.tank(0,0);
                    m_elevator.place();
                }
    
    
                break;

                case 5:

                if(m_drivetrain.getEncoder() > -20){
                    driveForward(-.65, -30, -80,-100);
                }else{
                    if(driveForward(-.7, 20, -115,-130)){
                        m_autoStep++;
                        ticks = 0;
                    }
                }
                
                break;

                case 6:

                ticks++;

                m_drivetrain.turnTo(180);
                if(m_drivetrain.getAngle() > 178 && ticks > 30){
                    m_autoStep++;
                    ticks = 0;
                }

                break;

                case 7:

                ticks++;
                if(m_drivetrain.target(0) && ticks > 15){
                    m_drivetrain.tank(0,0);
                    m_autoStep++;
                    m_drivetrain.resetEncoders();
                }

                break;

                case 8:

                if(driveForward(-.7, 195, -90,-100)){
                    m_autoStep++;
                    ticks = 0;
                }

                break;

                case 9:

                m_drivetrain.turnTo(310);
                m_elevator.setLevel(LiftLevels.HATCH_HIGH);

                if(m_drivetrain.getAngle() > 307){
                    m_autoStep++;
                    ticks = 0;
                }

                break;

                case 10:

                ticks++;
                if(m_drivetrain.target(-30) && ticks > 30){
                    m_autoStep++;
                    m_drivetrain.tank(0,0);
                }

                break;

                case 11:

                break;
            }
        }else{
            
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
    }

    int pressState = 0;
    int grabState = 0;
    public void teleopPeriodic() {     
        
        m_drivetrain.arcadeDrive(m_joystick.getRawAxis(1), m_joystick.getRawAxis(2));

        if(m_joystick.getRawButton(4)){ 
            m_drivetrain.target(-30);
            return;
        }

        // Switch statement controlling what object the robot is grabbing
        switch(pressState){
            case 0:
                pressState = 0;
                if(m_joystick.getRawButton(3) && m_elevator.getRaw() < 10)
                    pressState = 1;
                break;

            case 1:
                
                pressState = 1;

                if(!m_joystick.getRawButton(3)){
                    pressState = 2;
                    m_robotState.setState(State.GRAB_CARGO);
                }
                break;

            case 2:

                pressState = 2;
                
                if(m_joystick.getRawButton(3))
                    pressState = 3;
                break;

            case 3:

                pressState = 3;
                
                if(!m_joystick.getRawButton(3)){
                    pressState = 0;
                    m_robotState.setState(State.PLACE_PANEL);
                }
                break;
        }
        
        if(m_robotState.state() == State.PLACE_PANEL){

            // ELEVATOR CONTROLS
            if(m_joystick.getRawButton(7))
                m_elevator.setLevel(LiftLevels.HATCH_HIGH);
            if(m_joystick.getRawButton(9))
                m_elevator.setLevel(LiftLevels.HATCH_MEDIUM);
            if(m_joystick.getRawButton(11))
                m_elevator.setLevel(LiftLevels.HATCH_LOW);
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

        // GENERAL CONTROLS

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

        // RUNS PASSTHROUGH SYSTEM
        if(m_joystick.getRawButton(10)){
            m_cargoSystem.runSystem();
        }

        // MANUAL FUNCTIONS

        // Reverse Passthrough
        if(m_nav.getRawButton(1)){
            m_cargoSystem.runPassthrough(-1);;
        }

        // Reset Passthrough
        if(m_nav.getRawButton(4)){
            pressState = 0;
            m_robotState.setState(State.PLACE_PANEL);
        }     
    }

    public void disabledInit(){
        m_drivetrain.tank(0,0);
        m_elevator.reset();
        
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
    }
}
