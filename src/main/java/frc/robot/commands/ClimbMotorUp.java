// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kernal.RobotContainer;
import frc.robot.subsystems.Climb;

public class ClimbMotorUp extends Command {
  /** Creates a new ClimbMotorUp. */
  public ClimbMotorUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Climb.runCMR(0.50);
    Climb.runCML(0.50);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climb.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
