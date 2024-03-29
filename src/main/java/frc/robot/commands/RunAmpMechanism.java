// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kernal.RobotContainer;
import frc.robot.subsystems.AmpMechanism;

public class RunAmpMechanism extends Command {
  /** Creates a new RunAmpMechanism. */
  public RunAmpMechanism() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.amp_mechanism);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AmpMechanism.runAmpMechanism();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AmpMechanism.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
