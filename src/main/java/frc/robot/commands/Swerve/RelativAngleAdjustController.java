// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;


import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class RelativAngleAdjustController extends Command {
  private static PIDController pid;

  private ChassisSpeeds speeds = new ChassisSpeeds();
  private double omega;

  private LoggedDouble toleranceLog;
  private LoggedDouble omegaLog;
  private LoggedBool atPointLog;

  public static boolean atPoint() {
    return pid.atSetpoint();
  }

  public RelativAngleAdjustController() {
    pid = new PIDController(
      SwerveConstants.RELATIV_THATA_KP,
      SwerveConstants.RELATIV_THATA_KI,
      SwerveConstants.RELATIV_THATA_KD
    );

    toleranceLog = new LoggedDouble("/Swerve/Controllers/Relativ Adjust/Tolerance");
    omegaLog = new LoggedDouble("/Swerve/Controllers/Relativ Adjust/Omega Speed");
    atPointLog = new LoggedBool("/Swerve/Controllers/Relativ Adjust/At Point");
    pid.setTolerance(SwerveConstants.RELATIV_ANGLE_PID_TOLORANCE);
    pid.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setTolerance(getTolerance());
    omega = pid.calculate(Vision.getInstance().getTx());
    speeds.omegaRadiansPerSecond = omega;

    toleranceLog.update(getTolerance());
    omegaLog.update(omega);
    atPointLog.update(getAtPoint());
  }

  public double getTolerance() {
    return 20 / RobotConstants.SUPER_STRUCTURE.getDistanceToTag();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speeds;
  }

  public boolean getAtPoint() {
    return pid.atSetpoint() && Vision.getInstance().isTag() &&  (Vision.getInstance().getTagID() == 7 ||
    Vision.getInstance().getTagID() == 4);
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