// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  static WPI_TalonSRX ConveyorMotor1 = new WPI_TalonSRX(Constants.conveyorID_1);
  static WPI_TalonSRX ConveyorMotor2 = new WPI_TalonSRX(Constants.conveyorID_2);

  /** Creates a new Conveyor. */
  public Conveyor() {}

  public static void run(double speed) {
    ConveyorMotor1.set(ControlMode.PercentOutput, -speed);
    ConveyorMotor2.set(ControlMode.PercentOutput, speed);
  }

  public static void cancel() {
    ConveyorMotor1.set(ControlMode.PercentOutput, 0);
    ConveyorMotor2.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
