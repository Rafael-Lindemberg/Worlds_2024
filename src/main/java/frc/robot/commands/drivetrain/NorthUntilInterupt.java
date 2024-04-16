// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class NorthUntilInterupt extends Command {
  private Rotation2d m_setAngle;
  private final DriveTrain drivetrain;
  private final DoubleSupplier forward, strafe;
  BooleanSupplier interrupt;

  private final ProfiledPIDController angleController;

  /** Creates a new NorthUntilInterupt. */
  public NorthUntilInterupt(DriveTrain drivetrain, DoubleSupplier strafe, DoubleSupplier forward, BooleanSupplier interrupt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.interrupt = interrupt;

    // angleController = new ProfiledPIDController(3, 0.1, 0, 
    //   new TrapezoidProfile.Constraints(
    //     drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
    //     drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
    //   )
    // );
    // angleController.enableContinuousInput(0, 2*Math.PI);
    angleController = DriveTrain.getTunedRotationalPIDControllerForHolonomicDrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Robot.isRedAlliance()){
      m_setAngle = Rotation2d.fromDegrees(180);
    }else{
      m_setAngle = new Rotation2d();
    }
    
    angleController.reset(drivetrain.getPose2d().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getController().setRumble(RumbleType.kBothRumble, 1.0);
    double f = forward.getAsDouble();
    double s = strafe.getAsDouble();
    
    Rotation2d currentAngle = drivetrain.getPose2d().getRotation();//new Rotation2d(Math.toRadians(DriveTrain._gyro.getAngle()));
    Rotation2d targetAngle = m_setAngle;
    //targetAngle = (targetAngle == null) ? previousTarget : targetAngle;
  
    double t = angleController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
    drivetrain.holonomicDrive( -f, -s, -t, true);
    //System.out.println("current angle : " + currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("North Command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return interrupt.getAsBoolean();
  }
}
