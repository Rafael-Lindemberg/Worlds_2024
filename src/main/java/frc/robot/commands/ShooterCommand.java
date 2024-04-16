// ShooterCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooter;

    public ShooterCommand(ShooterSubsystem shooterSubsystem) {
        shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {  
        shooter.shoot(1); 
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
