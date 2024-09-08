// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class RelativAngleAdjustController extends Command {
  private static PIDController pid;

  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private Supplier<Double> ty;
  private boolean useGyro;
  private ChassisSpeeds speeds;

  public static boolean atPoint() {
    return pid.atSetpoint();
  }

  public RelativAngleAdjustController() {
    pid = new PIDController(
      SwerveConstants.RELATIV_THATA_KP,
      SwerveConstants.RELATIV_THATA_KI,
      SwerveConstants.RELATIV_THATA_KD
    );
    pid.setTolerance(SwerveConstants.RELATIV_ANGLE_PID_TOLORANCE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setSetpoint(0);
    Supplier<Double> getMeserment = () -> Vision.getInstance().getTx();
    speeds = new ChassisSpeeds(0, 0, pid.calculate(getMeserment.get()));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}