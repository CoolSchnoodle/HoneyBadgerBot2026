// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleConsumer;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX leftLeader;
  private final TalonFX leftFollower;
  private final TalonFX rightLeader;
  private final TalonFX rightFollower;

  private final DifferentialDrive drive;

  public DriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new TalonFX(LEFT_LEADER_ID);
    leftFollower = new TalonFX(LEFT_FOLLOWER_ID);
    rightLeader = new TalonFX(RIGHT_LEADER_ID);
    rightFollower = new TalonFX(RIGHT_FOLLOWER_ID);

    // set up differential drive class
    DoubleConsumer setLeft = (double leftSpeed) -> {
      leftLeader.set(leftSpeed);
      leftFollower.set(leftSpeed);
    };
    DoubleConsumer setRight = (double rightSpeed) -> {
      rightLeader.set(rightSpeed);
      rightFollower.set(rightSpeed);
    };
    drive = new DifferentialDrive(setLeft, setRight);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    TalonFXConfiguration leftConfig = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT))
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake));
    TalonFXConfiguration rightConfig = new TalonFXConfiguration()
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT))
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake));
    leftLeader.getConfigurator().apply(leftConfig);
    leftFollower.getConfigurator().apply(leftConfig);
    rightLeader.getConfigurator().apply(rightConfig);
    rightFollower.getConfigurator().apply(rightConfig);
}

  @Override
  public void periodic() {
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

}
