// ShooterCommand.java
package frc.robot.commands.AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShooterMid extends Command {
    private final ShooterSubsystem shooter;

    public AmpShooterMid(ShooterSubsystem shooterSubsystem) {
        shooter = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
       
        shooter.shoot(0.1125); 
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
