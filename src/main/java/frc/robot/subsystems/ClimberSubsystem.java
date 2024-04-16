package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    private final com.ctre.phoenix6.hardware.TalonFX  rightClimberMotor;
    private final com.ctre.phoenix6.hardware.TalonFX  leftClimberMotor;

       public ClimberSubsystem() {

         rightClimberMotor = new TalonFX(17);
         leftClimberMotor = new TalonFX(18);
    }

    public void setRightClimberDown(double speed) {   
        rightClimberMotor.set(speed*-1);
    }

    public void setLeftClimberDown(double speed){
        leftClimberMotor.set(speed*-1);
    }

    
    public void setRightClimberUp(double speed) {   
        rightClimberMotor.set(speed);
    }

    public void setLeftClimberUp(double speed){
        leftClimberMotor.set(speed);
    }

    public void stopRight() {
        rightClimberMotor.set(0);
    }

        public void stopLeft() {
        leftClimberMotor.set(0);
    }
}


