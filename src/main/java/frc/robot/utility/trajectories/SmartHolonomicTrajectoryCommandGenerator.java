// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.trajectories;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/** Generates command that follows an optimized trajectory to the desired end point based on the robot's current position. */
public class SmartHolonomicTrajectoryCommandGenerator {
    private SmartHolonomicTrajectoryCommandGenerator() {}

    public static Command toCommunityZone(DriveTrain drivetrain) {
        ProfiledPIDController headingController = DriveTrain.getTunedRotationalPIDController();
        headingController.enableContinuousInput(-Math.PI, Math.PI); // why -pi to pi?

        Command command = new SmartHolonomicTrajectoryCommand(
            () -> SmartHolonomicTrajectoryGenerator.toCommunityZone(drivetrain),
            drivetrain::getPose2d,
            DriveTrain.getTunedTranslationalPIDController(),
            DriveTrain.getTunedTranslationalPIDController(),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command.andThen(drivetrain::stop);
    }

    public static Command toLoadingStation(DriveTrain drivetrain) {
        ProfiledPIDController headingController = DriveTrain.getTunedRotationalPIDController();
        headingController.enableContinuousInput(-Math.PI, Math.PI); // why -pi to pi?

        Command command = new SmartHolonomicTrajectoryCommand(
            () -> SmartHolonomicTrajectoryGenerator.toLoadingStation(drivetrain),
            drivetrain::getPose2d,
            DriveTrain.getTunedTranslationalPIDController(),
            DriveTrain.getTunedTranslationalPIDController(),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command.andThen(drivetrain::stop);
    }

    public static Command leaveCommunity(DriveTrain drivetrain) {
        ProfiledPIDController headingController = DriveTrain.getTunedRotationalPIDController();
        headingController.enableContinuousInput(-Math.PI, Math.PI); // why -pi to pi?

        Command command = new SmartHolonomicTrajectoryCommand(
            () -> SmartHolonomicTrajectoryGenerator.leaveCommunity(drivetrain),
            drivetrain::getPose2d,
            DriveTrain.getTunedTranslationalPIDController(),
            DriveTrain.getTunedTranslationalPIDController(),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command.andThen(drivetrain::stop);
    }
}
