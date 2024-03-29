// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpMechanism extends SubsystemBase {

  static WPI_TalonSRX AmpMotor = new WPI_TalonSRX(Constants.ampID);

  /** Creates a new AmpMechanism. */
  public AmpMechanism() {}

  public static void runAmpMechanism() {
    AmpMotor.set(ControlMode.PercentOutput, Constants.ampMechanismSpeed);
  }

  public static void cancel() {
    AmpMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
