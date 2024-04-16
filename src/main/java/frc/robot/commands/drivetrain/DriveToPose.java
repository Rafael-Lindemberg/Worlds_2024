// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveToPose extends Command {
  private final DriveTrain drivetrain;
  private final Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private Supplier<Boolean> interupt;
  
  private final PIDController xController, yController;
  private final ProfiledPIDController angleController;
  /**
   * Creates a command that drives to the desired pose using PID.
   * 
   * The path it uses to get there may be unpredictable, but it'll be a relatively straight line.
   * 
   * The supplier will only be checked once every time the command is initialized.
   * 
   * @param drivetrain the drivetrian to control
   * @param targetPoseSupplier the pose to drive to
   */
  public DriveToPose(DriveTrain drivetrain, Supplier<Pose2d> targetPoseSupplier, Supplier<Boolean> interupt) {
    addRequirements(drivetrain);
    this.interupt = interupt;
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.targetPose = new Pose2d();
    // xController = new PIDController(1.8, 0.5, 0);
    // yController = new PIDController(2, 0.6, 0);
    xController = DriveTrain.getTunedTranslationalPIDController();
    yController = DriveTrain.getTunedTranslationalPIDController();
    // angleController = new ProfiledPIDController(3, 0.8, 0,
    //   new TrapezoidProfile.Constraints(
    //     drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
    //     drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
    //   )
    // );
    // angleController.enableContinuousInput(0, 2*Math.PI);
    // angleController.setTolerance(Math.toRadians(1)); // How accurate should the angle be before stopping?
    angleController = DriveTrain.getTunedRotationalPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.reset(drivetrain.getPose2d().getRotation().getRadians());
    xController.reset();
    yController.reset();
    targetPose = targetPoseSupplier.get();
  }

  private double capValue(double value, double cap) {
    if (value > cap){return cap;}else if (value < -cap){return -cap;} else {return value;}
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetPose == null) {
      return;
    }
    Pose2d currentPose = drivetrain.getPose2d();
    double xFeedback = capValue(xController.calculate(currentPose.getX(), targetPose.getX()), 1.8);
    double yFeedback = capValue(yController.calculate(currentPose.getY(), targetPose.getY()), 1.8);
    //double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    //double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaFF = angleController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    drivetrain.fromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFF, drivetrain.getPose2d().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(targetPose == null || interupt.get()){
      return true;
    }
    return (xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint());
  }
}