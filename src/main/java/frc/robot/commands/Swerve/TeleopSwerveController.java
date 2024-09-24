// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ma5951.utils.Logger.LoggedString;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class TeleopSwerveController extends Command {
  
  private DriveController driveCommand;
  private AngleAdjustController angleAdjustCommand;
  private RelativAngleAdjustController relativAngleAdjustCommand;
  private IntakeAutoDriveController intakeAutoDriveCommand;
  private ChassisSpeeds driveControllerSpeeds;
  private ChassisSpeeds angleAdjustControllerSpeeds;
  private ChassisSpeeds relativAngleAdjustControllerSpeeds;
  private ChassisSpeeds intakeAutoDriveSpeeds;
  private boolean isOdometry;
  public static boolean atPoint;

  private SwerveSubsystem swerve;
  private ChassisSpeeds robotSpeeds;
  private LoggedString xyControllerLog;
  private LoggedString theathControllerLog;
  
  public TeleopSwerveController(CommandPS5Controller controller) {
    swerve = SwerveSubsystem.getInstance();
    
    driveCommand = new DriveController(controller);
    angleAdjustCommand = new AngleAdjustController(true);
    relativAngleAdjustCommand = new RelativAngleAdjustController();
    intakeAutoDriveCommand = new IntakeAutoDriveController(controller);
    xyControllerLog = new LoggedString("/Swerve/Controllers/XY Controller");
    theathControllerLog = new LoggedString("/Swerve/Controllers/Theath Controller");
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    driveCommand.initialize();
    angleAdjustCommand.initialize();
    relativAngleAdjustCommand.initialize();
  }

  public boolean getAtPoint() {
    return angleAdjustCommand.getAtPoint() || (relativAngleAdjustCommand.getAtPoint() && (Vision.getInstance().getTagID() == 7 ||
    Vision.getInstance().getTagID() == 4));
  }

  @Override
  public void execute() {
    driveCommand.execute();
    angleAdjustCommand.execute();
    relativAngleAdjustCommand.execute();
    intakeAutoDriveCommand.execute();
    driveControllerSpeeds = driveCommand.getChassisSpeeds();
    angleAdjustControllerSpeeds = angleAdjustCommand.getChassisSpeeds();
    relativAngleAdjustControllerSpeeds = relativAngleAdjustCommand.getChassisSpeeds();
    intakeAutoDriveSpeeds = intakeAutoDriveCommand.getChassisSpeeds();

    if (RobotContainer.currentRobotState == RobotConstants.AMP) {
      angleAdjustCommand.setSetPoint(RobotConstants.SUPER_STRUCTURE.getSetPointForAmpAline());
    } else {
      angleAdjustCommand.setSetPoint(RobotConstants.SUPER_STRUCTURE.getSetPointForAline());
    }
    atPoint = angleAdjustCommand.getAtPoint() || relativAngleAdjustCommand.getAtPoint();

    if ((RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING && (Vision.getInstance().isTag() && Vision.getInstance().getTagID() == 7
    || Vision.getInstance().getTagID() == 4 ) && !isOdometry )){
      robotSpeeds = new ChassisSpeeds(0 , 0, relativAngleAdjustControllerSpeeds.omegaRadiansPerSecond);
      xyControllerLog.update("Drive Controller");
      theathControllerLog.update("Relativ Adjust");
    } else if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING ){
      xyControllerLog.update("Drive Controller");
      theathControllerLog.update("Odometry Adjust Speaker");
      // if (RobotConstants.SUPER_STRUCTURE.getDistanceToTag() < RobotConstants.DISTANCE_TO_SHOOT ) {
      //   isOdometry = true;
      // }
      robotSpeeds = new ChassisSpeeds(0 , 0, angleAdjustControllerSpeeds.omegaRadiansPerSecond);
    } else if (RobotContainer.currentRobotState == RobotConstants.AMP && RobotConstants.SUPER_STRUCTURE.isNote()){
      robotSpeeds = new ChassisSpeeds(driveControllerSpeeds.vxMetersPerSecond * 0.5 , driveControllerSpeeds.vyMetersPerSecond * 0.5, angleAdjustControllerSpeeds.omegaRadiansPerSecond);
      driveCommand.updateAngleToLock();
      xyControllerLog.update("Drive Controller");
      theathControllerLog.update("Odometry Adjust Amp");
    } else if (RobotContainer.driverController.getHID().getR1Button()) {
      robotSpeeds = intakeAutoDriveSpeeds;
      xyControllerLog.update("Intake Auto Drive");
      theathControllerLog.update("Intake Auto Drive");
    } else { 
      xyControllerLog.update("Drive Controller");
      theathControllerLog.update("Drive Controller");
      robotSpeeds = driveControllerSpeeds;
      isOdometry = false;
    }

    swerve.drive(robotSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
