// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;

import com.ma5951.utils.Vision.Limelight3G;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private Limelight3G limelight;

  private Vision() {
    limelight = new Limelight3G(VisionConstants.CAMERA_NAME,
     VisionConstants.CAMERA_HIGHT,
      VisionConstants.CAMERA_ANGLE);
  }

  private boolean isTagSpeaker() {
    return limelight.getTagId() == 7 || limelight.getTagId() == 4;
  }

  public double getTySpeaker() {
    if (isTagSpeaker()) {
      return limelight.getY();
    } 
    return 0;
  }

  public double getDistanceToSpraker() {
    if (isTagSpeaker()) {
      return limelight.distance();
    } 
    return 0;
  }

  public Pose2d getEstiman() {
    return limelight.getEstPose();
  }

  public double getLatency() {
    return limelight.getL();
  }

  public double getTimeStamp() {
    return limelight.getTimeStamp();
  }

  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
  }

  @Override
  public void periodic() {
  }

}
