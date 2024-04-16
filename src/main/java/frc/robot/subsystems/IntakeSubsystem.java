package frc.robot.subsystems;

//change to kraken
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX positiveMotor;
    private final TalonFX negativeMotor;
    private final CANSparkMax lowMotor;

    public IntakeSubsystem() {
        positiveMotor = new TalonFX(13);
        negativeMotor = new TalonFX(14);
        lowMotor = new CANSparkMax(19, com.revrobotics.CANSparkLowLevel.MotorType.kBrushed);
    }

    public void intake(double speed, double speedTwo, double speedThree) {   
        positiveMotor.set(-speed);
        negativeMotor.set(-speedTwo);
        lowMotor.set(-speed);
    }

    public void stop() {
        positiveMotor.set(0);
        negativeMotor.set(0);
        lowMotor.set(0);
    }
}