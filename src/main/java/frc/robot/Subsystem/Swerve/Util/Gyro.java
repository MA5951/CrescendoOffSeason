package frc.robot.Subsystem.Swerve.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Gyro {

    void reset();

    double getPitch();

    double getRoll();

    double getYaw();

    double getAbsYaw();

    double getAccelX();

    double getAccelY();

    void update(ChassisSpeeds robotSpeeds);

}

