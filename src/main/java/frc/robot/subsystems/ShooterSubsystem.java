package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final com.ctre.phoenix6.hardware.TalonFX positiveMotor; // TalonFX 
    private final com.ctre.phoenix6.hardware.TalonFX negativeMotor; 

    public ShooterSubsystem() {
        positiveMotor = new TalonFX(15);
        negativeMotor = new TalonFX(16);
    }

    public void shoot(double speed) {   
        positiveMotor.set(speed);
        negativeMotor.set((speed*-1));
    }

    public void stop() {
        positiveMotor.set(0);
        negativeMotor.set(0);
    }
}
