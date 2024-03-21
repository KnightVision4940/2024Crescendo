// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.kernal.RobotContainer;
import frc.robot.subsystems.Intake;

public class OuttakeAuto extends Command {
  double m_startTime;

  /** Creates a new RunIntake. */
  public OuttakeAuto() {
    addRequirements(RobotContainer.intake);
    // Use addRequirements() here to declare subsystem dependencies.
    System.out.println("outtake auto");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.run(0.65, -0.40);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    // double currentTime = RobotController.getFPGATime() / 1000.0;
    if ((currentTime - m_startTime) <= Constants.outtakeTime) {
      return false;
    } else {
      return true;
    }
  }
}
