/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ma5951.utils.Vision;

import java.util.function.Supplier;

import com.ma5951.utils.Utils.DriverStationUtil;
import com.ma5951.utils.Vision.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.LimelightHelpers.RawDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight3G {
  private String name;
  private double cammeraHight;
  private double cammeraAngle;
  private double offset = 0;
  private Supplier<Double> robotAngleSupplier;


  public Limelight3G(
    String cammeraName , double cammeraHight , double cammeraAngle , Supplier<Double> angleSupplier){
      name  = cammeraName;
      this.cammeraHight = cammeraHight;
      this.cammeraAngle = cammeraAngle;
      robotAngleSupplier = angleSupplier;
  }

  public LimelightHelpers.PoseEstimate getEstimatedPose() {
    if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) != null) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    } else {
      return new PoseEstimate();
    }
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

  public void filterTags(int[] tagsArry) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name , tagsArry);
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

  
  /*
   * Trigo
   */
  public double distance(double[] tagHights) {
    if (getTagID() >= 0 && getTagID() - 1 < 0) {
      return -1;
    }
    double deltaHight = tagHights[6] - cammeraHight;
    double deltaAngle = getTy() + cammeraAngle; 
    return deltaHight / Math.tan(Math.toRadians(deltaAngle));
  }

  public double getTx() {
    if (isTarget()) {
      return LimelightHelpers.getTX(name);
    } else {
      return 0;
    }
  }

  public double getTy() {
    if (isTarget()) {
      return LimelightHelpers.getTY(name);
    } else {
      return 0;
    }
  }

  public int getTagID() {
    return ((int)LimelightHelpers.getFiducialID(name));
  }

  public void resetHeading() {
    offset = robotAngleSupplier.get();
  }

  public void update() {
    if (DriverStationUtil.getAlliance() == Alliance.Red) {
      LimelightHelpers.SetRobotOrientation(name, robotAngleSupplier.get() - offset , 0, 0, 0, 0, 0);
    } else {
    LimelightHelpers.SetRobotOrientation(name, robotAngleSupplier.get() +180 - offset, 0, 0, 0, 0, 0);
  }
}
}