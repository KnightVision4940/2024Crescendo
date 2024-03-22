// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.kernal.RobotContainer;

public class Intake extends SubsystemBase {

  static WPI_TalonSRX RollerMotor = new WPI_TalonSRX(Constants.intakeID);

  static WPI_TalonSRX ConveyorMotor1 = new WPI_TalonSRX(Constants.conveyorID_1);
  static WPI_TalonSRX ConveyorMotor2 = new WPI_TalonSRX(Constants.conveyorID_2);

  /** Creates a new Intake. */
  public Intake() {}

  public static void run(double intakeSpeed, double conveyorSpeed) {
    RollerMotor.set(ControlMode.PercentOutput, intakeSpeed);

    ConveyorMotor1.set(ControlMode.PercentOutput, -conveyorSpeed);
    ConveyorMotor2.set(ControlMode.PercentOutput, conveyorSpeed);
  }

  public static void cancel() {
    RollerMotor.set(ControlMode.PercentOutput, 0);

    ConveyorMotor1.set(ControlMode.PercentOutput, 0);
    ConveyorMotor2.set(ControlMode.PercentOutput, 0);
  }

  public static void rumbleOnSensor() {
    RobotContainer.driveController.setRumble(RumbleType.kBothRumble, 0.3);
  }

  public static void cancelRumble() {
    RobotContainer.driveController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
