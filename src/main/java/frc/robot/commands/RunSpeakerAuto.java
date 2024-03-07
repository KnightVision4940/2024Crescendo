// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.kernal.RobotContainer;
import frc.robot.subsystems.Shooter;

public class RunSpeakerAuto extends Command {
  double m_startTime;

  /** Creates a new RunSpeaker. */
  public RunSpeakerAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = RobotController.getFPGATime() / 1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.run(0.50, 0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = RobotController.getFPGATime() / 1000.0;
    if ((currentTime - m_startTime) <= Constants.speakerTime) {
      return false;
    } else {
      return true;
    }
  }
}
