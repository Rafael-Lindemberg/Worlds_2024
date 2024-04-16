// ShooterCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends Command {
    private final IntakeSubsystem intake;

    public ReverseIntake(IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intake.intake(-0.7,-1,0); 
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when command ends
        intake.stop();
    }
}
