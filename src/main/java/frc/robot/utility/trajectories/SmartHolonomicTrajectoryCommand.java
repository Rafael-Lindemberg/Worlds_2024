// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.trajectories;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team1891.common.trajectory.HolonomicTrajectory;
import frc.team1891.common.trajectory.RotatingHolonomicDriveController;
import frc.team1891.common.trajectory.HolonomicTrajectory.State;

/** 
 * A command that follows a {@link HolonomicTrajectory}.
 */
public class SmartHolonomicTrajectoryCommand extends Command {
    private final Timer m_timer = new Timer();
    private final Supplier<HolonomicTrajectory> m_trajectorySupplier;
    private HolonomicTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RotatingHolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_outputChassisSpeeds;
    // private final Supplier<Rotation2d> m_desiredRotation;

    private final Field2d m_field = new Field2d();

    private boolean invalidTrajectory = false;

    /**
     * Constructs a new SwerveControllerCommand that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
     * @param requirements The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public SmartHolonomicTrajectoryCommand(
            Supplier<HolonomicTrajectory> trajectorySupplier,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Subsystem... requirements) {
        m_trajectorySupplier = trajectorySupplier;
        m_pose = pose;

        m_controller =
            new RotatingHolonomicDriveController(
                xController,
                yController,
                thetaController
            );

        m_outputChassisSpeeds =
            outputChassisSpeeds;

        addRequirements(requirements);

        SmartDashboard.putData("Holonomic Trajectory (Field2d)", m_field);
    }

    @Override
    public void initialize() {
        try {
            m_trajectory = m_trajectorySupplier.get();
        } catch (Exception exception) {
            DriverStation.reportWarning("The attempted trajectory command was aborted. The robot attempted to generate an invalid trajectory: " + exception.getMessage(), false);
            invalidTrajectory = true;
            return;
        }

        m_timer.reset();
        m_timer.start();

        m_field.getObject("Holonomic Trajectory Path").setTrajectory(m_trajectory.getAsTrajectory());

        m_controller.reset();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        if (invalidTrajectory) {
            return;
        }
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds =
            m_controller.calculate(m_pose.get(), desiredState);
        
        m_outputChassisSpeeds.accept(targetChassisSpeeds);

        m_field.setRobotPose(m_pose.get());
        m_field.getObject("Desired State").setPose(desiredState.poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_outputChassisSpeeds.accept(new ChassisSpeeds());
        invalidTrajectory = false;
    }

    @Override
    public boolean isFinished() {
        return invalidTrajectory || m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
  }
