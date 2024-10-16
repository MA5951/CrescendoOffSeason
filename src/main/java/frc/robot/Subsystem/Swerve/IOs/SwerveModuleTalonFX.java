package frc.robot.Subsystem.Swerve.IOs;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;

public class SwerveModuleTalonFX implements SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteEcoder;

    private TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();

    private boolean isDriveMotorReversed;
    private boolean isTurningMotorReversed;

    private MotionMagicVoltage turnController = new MotionMagicVoltage(0);
    private PositionVoltage pidTurnController = new PositionVoltage(0);
    private VelocityVoltage driveController = new VelocityVoltage(0);



    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> driveCurrent;
    private StatusSignal<Double> driveVolts;
    private StatusSignal<Double> driveTemp;
    private StatusSignal<Double> steerTemp;
    private StatusSignal<Double> steerPosition;
    private StatusSignal<Double> steerCurrent;
    private StatusSignal<Double> steerVolts;
    private StatusSignal<Double> absAngle;
    private StatusSignal<Double> steerVelocitt;
    private String moduleName;

    private LoggedDouble DrivePosition;
    private LoggedDouble DriveVelocity;
    private LoggedDouble DriveCurrent;
    private LoggedDouble DriveVolts;
    private LoggedDouble SteerPosition;
    private LoggedDouble SteerCurrent;
    private LoggedDouble SteerVolts;
    private LoggedDouble AbsAngle;
    private LoggedDouble velociSteer;
    private LoggedDouble DriveTemp;
    private LoggedDouble SteerTemp;

    

    public SwerveModuleTalonFX(String moduleNameN , int driveID,
            int turningID, int absoluteEncoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed, String canbus) {
        
        this.driveMotor = new TalonFX(driveID, canbus);
        this.turningMotor = new TalonFX(turningID, canbus);
        this.absoluteEcoder = new CANcoder(absoluteEncoderID, canbus);
      

        this.isDriveMotorReversed = isDriveMotorReversed;
        this.isTurningMotorReversed = isTurningMotorReversed;
        this.moduleName = moduleNameN;

        
        DrivePosition = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Position");
        DriveVelocity = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Velocity");
        DriveCurrent = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Current");
        DriveVolts = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Volts");
        DriveTemp = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Temp");
        SteerTemp = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Temp");
        SteerPosition = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Position");
        SteerCurrent = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Current");
        SteerVolts = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Volts");
        AbsAngle = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Absolute Angle");
        velociSteer = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Velocity");
        

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveCurrent = driveMotor.getStatorCurrent();
        driveVolts = driveMotor.getMotorVoltage();
        driveTemp = driveMotor.getDeviceTemp();
        steerTemp = turningMotor.getDeviceTemp();
        steerPosition = turningMotor.getPosition();
        steerCurrent = turningMotor.getStatorCurrent();
        steerVolts = turningMotor.getMotorVoltage();
        absAngle = absoluteEcoder.getAbsolutePosition();
        steerVelocitt = turningMotor.getVelocity();

        
        
        

        configTurningMotor();
        configDriveMotor();
        //configCANCoder();
        resetSteer();

        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.ODOMETRY_UPDATE_RATE, 
        drivePosition , steerPosition , driveVelocity);
    }

    private void configTurningMotor() {
        //turningConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40; //80Deafult
        //turningConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40; //80Deafult
        turningConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turningConfiguration.Feedback.SensorToMechanismRatio = SwerveConstants.TURNING_GEAR_RATIO;

        turningConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        turningConfiguration.MotorOutput.Inverted = 
            isTurningMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        turningConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        turningConfiguration.Slot0.kP = SwerveConstants.TURNING_kP;
        turningConfiguration.Slot0.kI = SwerveConstants.TURNING_kI;
        turningConfiguration.Slot0.kD = SwerveConstants.TURNING_kD;
        //turningConfiguration.MotionMagic.MotionMagicCruiseVelocity = SwerveConstants.TURNING_CTUISE_VELOCITY;
        //turningConfiguration.MotionMagic.MotionMagicAcceleration = SwerveConstants.TURNING_ACCELERATION;
        //turningConfiguration.MotionMagic.MotionMagicJerk = SwerveConstants.TURNING_JERK;



        turningConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.TURNING_ENABLE_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentLimit =
            SwerveConstants.TURNING_CONTINUOUS_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentThreshold =
            SwerveConstants.TURNING_PEAK_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyTimeThreshold = 
            SwerveConstants.TURNING_PEAK_CURRENT_DURATION;
        
        turningMotor.getConfigurator().apply(turningConfiguration);

        
    }

    private void configDriveMotor() {
        //driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40; //80Deafult
        //driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40; //80Deafult

        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfiguration.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;

        driveConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

        driveConfiguration.MotorOutput.Inverted = 
            isDriveMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfiguration.Slot0.kP = SwerveConstants.DRIVE_kP_TELEOP;
        driveConfiguration.Slot0.kI = SwerveConstants.DRIVE_kI_TELEOP;
        driveConfiguration.Slot0.kD = SwerveConstants.DRIVE_kD_TELEOP;
        driveConfiguration.Slot0.kS = SwerveConstants.DRIVE_kS_TELEOP;
        driveConfiguration.Slot0.kV = SwerveConstants.DRIVE_kV_TELEOP;

        driveConfiguration.Slot1.kP = SwerveConstants.DRIVE_kP_AUTO;
        driveConfiguration.Slot1.kI = SwerveConstants.DRIVE_kI_AUTO;
        driveConfiguration.Slot1.kD = SwerveConstants.DRIVE_kD_AUTO;
        driveConfiguration.Slot1.kS = SwerveConstants.DRIVE_kS_AUTO;
        driveConfiguration.Slot1.kV = SwerveConstants.DRIVE_kV_AUTO;

        driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.DRIVE_ENBLE_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentLimit = 
            SwerveConstants.DRIVE_CONTINUOS_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentThreshold = 
            SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyTimeThreshold = 
            SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void resetSteer() {
        turningMotor.setPosition((getAbsoluteEncoderPosition() / 360 ));
    }

    public double getDriveTemp() {
        driveTemp.refresh();
        return driveTemp.getValueAsDouble();
    }

    public double getSteerTemp() {
        steerTemp.refresh();
        return steerTemp.getValueAsDouble();
    }

    public double getDriveCurrent() {
        driveCurrent.refresh();
        return driveCurrent.getValueAsDouble();
    }

    public double getSteerCurrent() {
        steerCurrent.refresh();
        return steerCurrent.getValueAsDouble();
    }

    public double getDriveVolts() {
        driveVolts.refresh();
        return driveVolts.getValueAsDouble();
    }

    public double getSteerVolts() {
        steerVolts.refresh();
        return steerVolts.getValueAsDouble();
    }

    public double getAbsoluteEncoderPosition() {
        absAngle.refresh();
        return absAngle.getValueAsDouble() * 360;
    }

    public double getDrivePosition() {
        //Return distance in meters
        drivePosition.refresh();
        return drivePosition.getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double getTurningPosition() {
        //Degrees
        steerPosition.refresh();
        return (steerPosition.getValueAsDouble() * 360);
    }

    public double getDriveVelocity() {
        driveVelocity.refresh();
        return (driveVelocity.getValueAsDouble() * 60) * Math.PI * (SwerveConstants.WHEEL_RADIUS * 2) / 60;
    }

    public void setNeutralModeDrive(Boolean isBrake) {
        driveConfiguration.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void setNeutralModeTurn(Boolean isBrake) {
        turningConfiguration.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turningMotor.getConfigurator().apply(turningConfiguration);
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    public void turningMotorSetVoltage(double volt) {
        turningMotor.setVoltage(volt);
    }

    public void driveMotorSetVoltage(double volt) {
        driveMotor.setVoltage(volt);
    }

    public void turningUsingPID(double setPointRdians) {
        //Degrees
        //turningMotor.setControl(turnController.withPosition(Units.radiansToRotations(setPointRdians)).withSlot(SwerveConstants.SLOT_CONFIG));
        turningMotor.setControl(pidTurnController.withPosition(Units.radiansToRotations(setPointRdians)).withSlot(0));
    }

    public void driveUsingPID(double setPointMPS) {
        //Meter Per Secound
        
        double omega = (setPointMPS / SwerveConstants.WHEEL_RADIUS);
        
        double rps = omega / (2 * Math.PI );
        if (DriverStation.isAutonomous()) {
            driveMotor.setControl(driveController.withVelocity(rps).withSlot(SwerveConstants.AUTO_SLOT_CONFIG));
        } else {
            driveMotor.setControl(driveController.withVelocity(rps).withSlot(SwerveConstants.TELEOP_SLOT_CONFIG));
        }

    }

    public void update() {
        
        DrivePosition.update(getDrivePosition());
        DriveVelocity.update(getDriveVelocity());
        DriveCurrent.update(getDriveCurrent());
        DriveVolts.update(getDriveVolts());
        SteerPosition.update(getTurningPosition() % 360);
        SteerCurrent.update(getSteerCurrent());
        SteerVolts.update(getSteerVolts());
        AbsAngle.update(getAbsoluteEncoderPosition());
        steerVelocitt.refresh();
        velociSteer.update(steerVelocitt.getValueAsDouble());
        DriveTemp.update(getDriveTemp());
        SteerTemp.update(getSteerTemp());

    }

}