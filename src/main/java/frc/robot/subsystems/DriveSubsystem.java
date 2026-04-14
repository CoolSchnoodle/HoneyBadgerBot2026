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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LocationUtils;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleConsumer;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX leftLeader;
  private final TalonFX leftFollower;
  private final TalonFX rightLeader;
  private final TalonFX rightFollower;

  private final DifferentialDrive drive;

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private PIDController rotationController = new PIDController(0.1, 0, 0);
  private Pose2d lastPose = null;

  public DriveSubsystem() {
    SmartDashboard.putString("Last limelight reading", "None");

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

    gyro.reset();
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    vision: {
      LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (estimate.tagCount == 0) {
        SmartDashboard.putString("Last limelight reading", "No tags");
        break vision;
      }
      if (
        estimate.tagCount == 1
        && estimate.rawFiducials[0].distToCamera > 3
      ) {
        SmartDashboard.putString("Last limelight reading", "Too far");
      }
      if (
        estimate.tagCount == 1
        && estimate.rawFiducials[0].ambiguity > 0.7
      ) {
        SmartDashboard.putString("Last limelight reading", "Too ambiguous");
        break vision;
      }
      if (estimate.pose == null) {
        SmartDashboard.putString("Last limelight reading", "null");
        break vision;
      }

      SmartDashboard.putString("Last limelight reading", "Good");
      lastPose = estimate.pose;
    }

  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
  public void driveArcade(double xSpeed, double zRotation, boolean squareInputs) {
    drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  class RotateToHubCommand extends Command {
    Rotation2d hubDir;
    DriveSubsystem driveSubsystem;

    public RotateToHubCommand(DriveSubsystem drive) {
      this.driveSubsystem = drive;
      addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
      if (driveSubsystem.lastPose == null) return;
      this.hubDir = LocationUtils.getDirectionToLocation(lastPose.getTranslation(), LocationUtils.getCurrentHubLocation().toTranslation2d());
    }

    @Override
    public void execute() {
      driveSubsystem.rotateTo(hubDir);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  public Command rotateToHubCommand() {
    return new RotateToHubCommand(this);
  }

  public void rotateTo(Rotation2d rotation) {
    drive.arcadeDrive(0, rotationController.calculate(gyro.getAngle(), rotation.getRadians()), false);
  }

  public Distance distanceToHub() {
    if (lastPose == null) return Meters.of(0);
    return LocationUtils.getDistanceToLocation(
      lastPose.getTranslation(),
      LocationUtils.getCurrentHubLocation().toTranslation2d()
    );
  }
}
