
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LeftClimberDown extends Command {
    private final ClimberSubsystem climber;

    public LeftClimberDown(ClimberSubsystem climberSubsystem){
        climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute(){
        climber.setLeftClimberDown(.7);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when command ends
        climber.stopLeft();
    }
}


