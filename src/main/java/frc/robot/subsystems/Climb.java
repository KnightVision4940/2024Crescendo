// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  static WPI_TalonSRX ClimbMotor1 = new WPI_TalonSRX(Constants.climbMotorID_1);
  static WPI_TalonSRX ClimbMotor2 = new WPI_TalonSRX(Constants.climbMotorID_2);

  /** Creates a new Climb. */
  public Climb() {}

  public static void run(double speed) {
    ClimbMotor1.set(ControlMode.PercentOutput, speed);
    ClimbMotor2.set(ControlMode.PercentOutput, -speed);
  }

  public static void cancel() {
    ClimbMotor1.set(ControlMode.PercentOutput, 0);
    ClimbMotor2.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
