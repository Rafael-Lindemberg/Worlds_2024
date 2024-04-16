package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class RightClimberUp extends Command {
    private final ClimberSubsystem climber;

    public RightClimberUp(ClimberSubsystem climberSubsystem){
        climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute(){
        climber.setRightClimberUp(.7);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when command ends
        climber.stopRight();
    }
}






