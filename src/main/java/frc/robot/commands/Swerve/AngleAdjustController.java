// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Swerve;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystem.Swerve.SwerveConstants;
// import frc.robot.Subsystem.Swerve.SwerveSubsystem;

// public class AngleAdjustController extends Command {
//   private static PIDController pid;

//   private SwerveSubsystem swerve;
//   private Supplier<Double> angle;
//   private boolean dontStopAtPoint = false;
//   private boolean useGyro;
//   private ChassisSpeeds speeds;

//   public static boolean atPoint() {
//     return pid.atSetpoint();
//   }

//   public AngleAdjustController(Supplier<Double> angle, boolean useGyro, boolean dontStopAtPoint) {
//     swerve = SwerveSubsystem.getInstance();
//     addRequirements(swerve);

//     this.useGyro = useGyro;
//     this.dontStopAtPoint = dontStopAtPoint;
    

//     pid = new PIDController(
//       SwerveConstants.THATA_KP,
//       SwerveConstants.THATA_KI,
//       SwerveConstants.THATA_KD
//     );
//     pid.setTolerance(SwerveConstants.ANGLE_PID_TOLORANCE);
//     this.angle = angle;
//     pid.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   public void setUseGyro(boolean use) {
//     useGyro = use;
//   }


//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     pid.setSetpoint(angle.get());
//     Supplier<Double> getMeserment = useGyro ? () -> Math.toRadians(swerve.getFusedHeading()) : .getRotation()::getRadians;
//     speeds = new ChassisSpeeds(0, 0, pid.calculate(getMeserment.get()));
//   }

//   public ChassisSpeeds getChassisSpeeds() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerve.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return pid.atSetpoint() && !dontStopAtPoint;
//   }
// }