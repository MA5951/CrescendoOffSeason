/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ma5951.utils.Vision;

import com.ma5951.utils.Vision.LimelightHelpers.RawDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight3G {
  private String name;


  public Limelight3G(
    String cammeraName){
      name  = cammeraName;
  }

  public LimelightHelpers.PoseEstimate getEstimatedPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  }

  public boolean isTarget() {
    return LimelightHelpers.getTV(name);
  }

  public LimelightHelpers.RawDetection getRawDetection() {
    if (isTarget()) {
      return LimelightHelpers.getRawDetections(name)[0]; //TODO
    } else {
      return new RawDetection(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  }

  public double getDistanceToTag() {
    if (isTarget()) {
      if (getEstimatedPose().rawFiducials.length > 0 ) {
        return getEstimatedPose().rawFiducials[0].distToRobot;
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  }

  public double getTx() {
    return LimelightHelpers.getTX(name);
  }

  public double getTagID() {
    return LimelightHelpers.getFiducialID(name);
  }
}