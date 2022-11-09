package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import frc.robot.subsystems.Limelight;

import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Arm extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController motorPID;
  private double desiredAngle = 0;
  private double defaultAngle = 1;
  /** Creates a new Arm. */
  public Arm() {
    motor = new CANSparkMax(ArmConstants.kArmMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor.setSmartCurrentLimit(80, 40);
    motor.setIdleMode(IdleMode.kBrake);
    encoder = motor.getEncoder();
    motorPID = motor.getPIDController();
    motorPID.setFF(ArmConstants.kArmFF);
    motorPID.setP(ArmConstants.kArmP, 0);
    motorPID.setD(ArmConstants.kArmD, 0);
    motorPID.setSmartMotionMaxVelocity(ArmConstants.kMaxVel, 0);
    motorPID.setSmartMotionMaxAccel(ArmConstants.kMaxAcc, 0);
    motorPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kPositionTolerance, 0);
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(ArmConstants.kRatio * 360);
    motor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kTopLimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kBottomLimit);
  }

  public void setTargetAngle(double targetAngle) {
    motorPID.setReference(targetAngle, ControlType.kSmartMotion, 0);
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public void set(double power) {
    motor.set(power);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void disable() {
    motor.set(0);
    setTargetAngle(defaultAngle);
  }


  public void homingTarget(boolean isHoming) {

    if (isHoming) {
      Limelight.enable();
    } else {
      Limelight.disable();
    }

    if (isHoming && Limelight.targetVisible()) {
      desiredAngle = encoder.getPosition() - Limelight.getDistanceToGoal();
    }
    else {
      desiredAngle = 0;
    }

    setTargetAngle(desiredAngle);

  }

  public boolean visionAligned() {
    if (Limelight.targetVisible() && Math.abs(Limelight.getYAngleOffset()) < ArmConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atDesiredAngle() {
    return Math.abs(desiredAngle - getPosition()) <= ArmConstants.kPositionTolerance;
  }

  public Command setAngle(double targetAngle) {
    return new StartEndCommand(
        () -> {
          setTargetAngle(targetAngle);
        },
        this::disable,
        this);
  }

  public Command levelOne() {
    return new InstantCommand(() -> setAngle(ArmConstants.kLevelOnePosition));
  }

  public Command levelTwo() {
    return new InstantCommand(() -> setAngle(ArmConstants.kLevelTwoPosition));
  }

  public Command levelThree() {
    return new InstantCommand(() -> setAngle(ArmConstants.kLevelThreePosition));
  }

  public Command homing(boolean isHoming) {
    return new StartEndCommand(
        () -> {
          homingTarget(isHoming);
        },
        this::disable,
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
