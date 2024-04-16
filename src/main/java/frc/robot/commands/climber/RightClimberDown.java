package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class RightClimberDown extends Command {
    private final ClimberSubsystem climber;

    public RightClimberDown(ClimberSubsystem climberSubsystem){
        climber = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute(){
        climber.setRightClimberDown(.7);
    }

    @Override
    public void end(boolean interrupted) {

        climber.stopRight();
    }
}




