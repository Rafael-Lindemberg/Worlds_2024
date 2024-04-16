// ShooterCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooter extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;

    public AutoShooter(ShooterSubsystem shooter, IntakeSubsystem intake) {
        m_ShooterSubsystem = shooter;
        m_IntakeSubsystem = intake;
        addRequirements(shooter);
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.shoot(1);
        m_IntakeSubsystem.intake(0.7,1,0); 
    }
        

    @Override
    public void end(boolean interrupted) {
        m_ShooterSubsystem.stop();
        m_IntakeSubsystem.stop();

    }
}
