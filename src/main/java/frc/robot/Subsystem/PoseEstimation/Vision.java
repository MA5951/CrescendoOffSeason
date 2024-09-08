// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Vision.Limelight3G;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private Limelight3G limelight;
  private LoggedDouble ditanceToSpeakerLog;
  private LoggedDouble tagIdLog;
  private LoggedDouble tXLog;
  private LoggedBool isTagLog;

  private Vision() {
    limelight = new Limelight3G(PortMap.Vision.CAMERA_NAME);

    ditanceToSpeakerLog = new LoggedDouble("/Vision/Distance To Speaker");
    tagIdLog = new LoggedDouble("/Vision/Tag Id");
    tXLog = new LoggedDouble("/Vision/Tx");
    isTagLog = new LoggedBool("/Vision/Is Tag");
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
    if (isTag()) {
      return limelight.getTagID();
    } else {
      return 0;
    }
  }

  public double getTx() {
    if (isTag()) {
      return limelight.getTx();
    } else {
      return 0;
    }
  }

  public double getDistanceToTag() {
    if (isTag()) {
      return limelight.getDistanceToTag();
    } else {
      return 0;
    }
  }

  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
  }

  @Override
  public void periodic() {
    ditanceToSpeakerLog.update(getDistanceToTag());
    tagIdLog.update(getTagID());
    tXLog.update(getTx());
    isTagLog.update(isTag());
  }

}
