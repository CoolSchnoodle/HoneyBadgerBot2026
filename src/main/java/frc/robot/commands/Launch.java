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

  public Launch(FuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    launcherSpeedAdjustment = Constants.FuelConstants.LAUNCHER_SPEED_ADJUSTMENT;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    fuelSubsystem.setLauncherPID(
      LAUNCHER_SPEED_ADJUSTMENT
      + SmartDashboard.getNumber("Launching launcher rotations per second", LAUNCHING_LAUNCHER_ROTATIONS_PER_SECOND));
    //fuelSubsystem
    //    .setIntakeLauncherRoller(
    //        SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
    if (launcherSpeedAdjustment == LAUNCHER_SPEED_ADJUSTMENT) {
      return;
    }
    launcherSpeedAdjustment = LAUNCHER_SPEED_ADJUSTMENT;
    double velocitySetpoint = SmartDashboard.getNumber("SpinUp launcher velocity setpoint", LAUNCHING_LAUNCHER_ROTATIONS_PER_SECOND) + launcherSpeedAdjustment;
    fuelSubsystem.setLauncherPID(velocitySetpoint);
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.setIntakeLauncherSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
