// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    DoubleConsumer setLeftPower = (double leftPower) -> { leftLeader.set(leftPower); leftFollower.set(leftPower); };
    DoubleConsumer setRightPower = (double rightPower) -> { rightLeader.set(rightPower); rightFollower.set(rightPower); };
    // set up differential drive class
    drive = new DifferentialDrive(
      setLeftPower,
      setRightPower
    );

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    // leftLeader.setCANTimeout(250);
    // rightLeader.setCANTimeout(250);
    // leftFollower.setCANTimeout(250);
    // rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    // SparkMaxConfig config = new SparkMaxConfig();
    // config.voltageCompensation(12);
    // config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    TalonFXConfiguration rightConfig = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT))
      .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    rightLeader.getConfigurator().apply(rightConfig);
    rightFollower.getConfigurator().apply(rightConfig);
    
    TalonFXConfiguration leftConfig = rightConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    leftLeader.getConfigurator().apply(leftConfig);
    leftFollower.getConfigurator().apply(leftConfig);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    // config.follow(leftLeader);
    // leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // config.follow(rightLeader);
    // rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    // config.disableFollowerMode();
    // rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    // config.inverted(true);
    // leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

}
