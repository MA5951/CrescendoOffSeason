// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class AngleAdjustController extends Command {
  private static PIDController pid;

  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private ChassisSpeeds speeds;
  private CommandPS5Controller controller;
  private boolean allowCorecttion = false;
  private double turningSpeed;
  private double omega;
  private LoggedDouble omegaLog;
  private LoggedBool atPointLog;

  public static boolean atPoint() {
    return pid.atSetpoint();
  }

  public void setAllowCorrection(boolean Allow) {
    allowCorecttion = Allow;
  }

  public AngleAdjustController(CommandPS5Controller controller) {
    this.controller = controller;

    pid = new PIDController(
      SwerveConstants.THATA_KP,
      SwerveConstants.THATA_KI,
      SwerveConstants.THATA_KD
    );
    omegaLog = new LoggedDouble("/Swerve/Controllers/Odometry Adjust/Omega Speed");
    atPointLog = new LoggedBool("/Swerve/Controllers/Odometry Adjust/At Point");
    pid.setTolerance(SwerveConstants.ANGLE_PID_TOLORANCE);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turningSpeed = controller.getRightX();
    turningSpeed = Math.abs(turningSpeed) < 0.1 ? 0 : -turningSpeed * 0.1;

    omega = pid.calculate(ConvUtil.DegreesToRadians(swerve.getFusedHeading() - 180));

    if (turningSpeed == 0 ) {
      speeds = new ChassisSpeeds(0, 0, omega);
    } else {
      speeds = new ChassisSpeeds(0, 0, omega + turningSpeed);
      pid.setSetpoint(swerve.getFusedHeading() - 180);
    }

    //speeds = new ChassisSpeeds(0, 0, omega);
    atPointLog.update(getAtPoint());
    omegaLog.update(omega);

  }

  public void setSetPoint(double setPoint) {
    pid.setSetpoint(setPoint);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speeds;
  }

  public boolean getAtPoint() {
    return pid.atSetpoint() ;
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