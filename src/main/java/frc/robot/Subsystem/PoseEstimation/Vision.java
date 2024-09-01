// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;

import com.ma5951.utils.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private Limelight limelight;

  public Vision() {
    limelight = new Limelight(VisionConstants.CAMERA_NAME,
     VisionConstants.CAMERA_HIGHT,
      VisionConstants.CAMERA_ANGLE);
  }

  public double getTySpeaker() {
    return limelight.
  }

  @Override
  public void periodic() {
  }

}
