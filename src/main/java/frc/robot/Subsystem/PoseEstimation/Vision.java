// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Vision.Limelight3G;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private Limelight3G limelight;
  private LoggedDouble ditanceToSpeakerLog;
  private LoggedPose2d poseLog;
  private LoggedDouble tagIdLog;
  private LoggedDouble tXLog;
  private LoggedBool isTagLog;

  private Vision() {
    limelight = new Limelight3G(PortMap.Vision.CAMERA_NAME , VisionConstants.CAMERA_HIGHT , VisionConstants.CAMERA_ANGLE
    ,() -> SwerveSubsystem.getInstance().getFusedHeading());

    ditanceToSpeakerLog = new LoggedDouble("/Vision/Distance To Speaker");
    tagIdLog = new LoggedDouble("/Vision/Tag Id");
    tXLog = new LoggedDouble("/Vision/Tx");
    isTagLog = new LoggedBool("/Vision/Is Tag");
    poseLog = new LoggedPose2d("/Vision/Pose");
  }

  public Pose2d getEstiman() {
    return limelight.getEstimatedPose().pose;
  }

  public double getTimeStamp() {
    return limelight.getEstimatedPose().timestampSeconds;
  }

  public boolean isTag() {
    return limelight.isTarget();
  }

  public double getTagID() {
    return limelight.getTagID();
  }

  public double getTx() {
    return limelight.getTx();
  }

  public double getDistanceToTag() {
    return limelight.getDistanceToTag();
  }

  public double getDistance() {
    return limelight.distance(VisionConstants.aprilTagsHights);
  }

  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
  }

  @Override
  public void periodic() {
    limelight.update();

    poseLog.update(getEstiman());
    ditanceToSpeakerLog.update(getDistance());
    tagIdLog.update(getTagID());
    tXLog.update(getTx());
    isTagLog.update(isTag());
  }

}
