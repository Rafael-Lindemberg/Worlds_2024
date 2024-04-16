// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1891.common.hardware.SimNavX;

public class WaitForNavX extends Command {
  SimNavX m_gyro;
  /** Creates a new WaitForNavX. */
  public WaitForNavX(SimNavX gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!m_gyro.isConnected() && !m_gyro.isCalibrating());
  }
}
