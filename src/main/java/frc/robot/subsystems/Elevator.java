
package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController motorPID;

  private double defaultPosition = 1;
  private double targetPosition = 0;

  public Elevator() {
    motor = new CANSparkMax(ElevatorConstants.kElevatorMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor.setSmartCurrentLimit(80, 40);
    motor.setIdleMode(IdleMode.kBrake);
    encoder = motor.getEncoder();
    motorPID = motor.getPIDController();
    motorPID.setFF(ElevatorConstants.kElevatorFF);
    motorPID.setP(ElevatorConstants.kElevatorP, 0);
    motorPID.setD(ElevatorConstants.kEleavtorD, 0);
    motorPID.setSmartMotionMaxVelocity(ElevatorConstants.kMaxVel, 0);
    motorPID.setSmartMotionMaxAccel(ElevatorConstants.kMaxAcc, 0);
    motorPID.setSmartMotionAllowedClosedLoopError(ElevatorConstants.kPositionTolerance, 0);
    encoder.setPosition(0);
    motor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.kTopLimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConstants.kBottomLimit);
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
    motorPID.setReference(targetPosition, ControlType.kSmartMotion, 0);
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
    setTargetPosition(defaultPosition);
  }

  public Command setPosition(double targetPosition) {
    return new StartEndCommand(
        () -> {
          setTargetPosition(targetPosition);
        },
        () -> {
          disable();
        });
  }

  @Override
  public void periodic() {

  }
}
