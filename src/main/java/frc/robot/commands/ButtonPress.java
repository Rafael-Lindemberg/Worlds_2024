package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ButtonPress extends Command {
  private String m_String; 

  public ButtonPress(String buttonState) {
    m_String = buttonState; 
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("CoGrab", m_String);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}