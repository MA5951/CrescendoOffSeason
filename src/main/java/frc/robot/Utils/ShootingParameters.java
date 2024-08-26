// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/** Add your docs here. */
public class ShootingParameters {

    private double leftSpeed;
    private double rightSpeed;
    private double armAngle;

    public ShootingParameters(double leftSpeed , double rightSpeed , double armAngle) {
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.armAngle = armAngle;
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public double getRightSpeed() {
        return rightSpeed;
    }

    public double getArmAngle() {
        return armAngle;
    }


}
