package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber {

    DoubleSolenoid m_front = new DoubleSolenoid(6, 0, 1);
    DoubleSolenoid m_back = new DoubleSolenoid(6, 2, 3);
    DoubleSolenoid m_stand = new DoubleSolenoid(6, 6, 7);
    //DoubleSolenoid m_stand = new DoubleSolenoid(12, 0, 1);

    public void lowerFront(){
        m_back.set(Value.kForward);
    }

    public void raiseFront(){
        m_back.set(Value.kReverse);
    }

    public void lowerStand(){
        m_stand.set(Value.kReverse);
    }

    public void raiseStand(){
        m_stand.set(Value.kForward);
    }

    public void lowerBack(){
        m_front.set(Value.kForward);
    }

    public void raiseBack(){
        m_front.set(Value.kReverse);
    }

}