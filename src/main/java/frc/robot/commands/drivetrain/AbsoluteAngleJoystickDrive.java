// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AbsoluteAngleJoystickDrive extends Command {
    private final DriveTrain drivetrain;
  private final Supplier<Rotation2d> rotation;
  // private Rotation2d previousTarget = new Rotation2d();
  private final DoubleSupplier forward, strafe;

  private final ProfiledPIDController angleController;

  /**
   * Creates a command that drives according to the joystick.
   * 
   * This differs from the normal {@link JoystickDrive} in that the angle is determined by an absolute, field relative, rotation supplier.
   * @param drivetrain
   * @param forward
   * @param strafe
   * @param rotation
   */
  public AbsoluteAngleJoystickDrive(DriveTrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, Supplier<Rotation2d> rotation) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = rotation;

    // // Theoretically this should be the same PID as fed to trajectories.
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
    angleController.reset(drivetrain.getPose2d().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double f = MathUtil.applyDeadband(forward.getAsDouble(), JoystickDrive.DEADBAND);
    double s = MathUtil.applyDeadband(strafe.getAsDouble(), JoystickDrive.DEADBAND);
    
    Rotation2d currentAngle = drivetrain.getPose2d().getRotation();
    Rotation2d targetAngle = rotation.get();
    //targetAngle = (targetAngle == null) ? previousTarget : targetAngle;
  
    double t = -angleController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

    drivetrain.holonomicDrive( f, s, t, true);

    // previousTarget = targetAngle;
    //System.out.printf("Desired angle %f Current angle %f%n", targetAngle.getDegrees(), currentAngle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
