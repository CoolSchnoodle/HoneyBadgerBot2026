// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {
  /** Creates a new Intake. */

  FuelSubsystem fuelSubsystem;
  double launcherSpeedAdjustment;
  double launchingSpeed;
  double feederVoltage;

  public Launch(FuelSubsystem fuelSystem, double launchingSpeed) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    launcherSpeedAdjustment = Constants.FuelConstants.LAUNCHER_SPEED_ADJUSTMENT;
    this.launchingSpeed = launchingSpeed;
    this.feederVoltage = SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
  }
  public Launch(FuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    launcherSpeedAdjustment = Constants.FuelConstants.LAUNCHER_SPEED_ADJUSTMENT;
    this.launchingSpeed = Constants.FuelConstants.LAUNCHING_LAUNCHER_ROTATIONS_PER_SECOND;
    this.feederVoltage = SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Launcher/Active", true);
    fuelSubsystem.setLauncherPID(
      launcherSpeedAdjustment + launchingSpeed);
    fuelSubsystem.setFeederRoller(0);
  }

  @Override
  public void execute() {
    double velocitySetpoint = launchingSpeed + launcherSpeedAdjustment;
    speedAdjust: {
      if (launcherSpeedAdjustment == LAUNCHER_SPEED_ADJUSTMENT) {
        break speedAdjust;
      }
      launcherSpeedAdjustment = LAUNCHER_SPEED_ADJUSTMENT;
      velocitySetpoint = launchingSpeed + launcherSpeedAdjustment;
      fuelSubsystem.setLauncherPID(velocitySetpoint);
    }
    double min = velocitySetpoint - SmartDashboard.getNumber("Launcher/Lower bound tolerance", LOWER_TOLERANCE);
    double max = velocitySetpoint + SmartDashboard.getNumber("Launcher/Upper bound tolerance", UPPER_TOLERANCE);
    if (fuelSubsystem.launchingLauncherReady(min, max)) {
      fuelSubsystem.setFeederRoller(feederVoltage);
    } else {
      fuelSubsystem.setFeederRoller(0);
    }
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.setIntakeLauncherSpeed(0);
    SmartDashboard.putBoolean("Launcher/Active", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
