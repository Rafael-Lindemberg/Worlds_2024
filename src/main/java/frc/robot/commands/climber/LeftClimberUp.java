package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class LeftClimberUp extends Command {
    private final ClimberSubsystem climber;

    public LeftClimberUp(ClimberSubsystem climberSubsystem){
        climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute(){
        climber.setLeftClimberUp(.7);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop shooter motors when co  mmand ends
        climber.stopLeft();
    }
}





