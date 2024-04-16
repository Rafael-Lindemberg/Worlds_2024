// ShooterCommand.java
package frc.robot.commands.AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShooter extends Command {
    private final ShooterSubsystem shooter;

    public AmpShooter(ShooterSubsystem shooterSubsystem) {
        shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
       
        shooter.shoot(0.10); 
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
