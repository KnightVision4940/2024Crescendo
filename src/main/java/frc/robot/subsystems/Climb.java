// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  static WPI_TalonSRX ClimbMotorLeft = new WPI_TalonSRX(Constants.climbMotorID_Left);
  static WPI_TalonSRX ClimbMotorRight = new WPI_TalonSRX(Constants.climbMotorID_Right);

  /** Creates a new Climb. */
  public Climb() {}

  public static void runCML(double speed1) {
    ClimbMotorLeft.set(ControlMode.PercentOutput, speed1);
  }

  public static void runCMR(double speed2) {
    ClimbMotorRight.set(ControlMode.PercentOutput, -speed2);
  }

  public static void cancel() {
    ClimbMotorLeft.set(ControlMode.PercentOutput, 0);
    ClimbMotorRight.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
