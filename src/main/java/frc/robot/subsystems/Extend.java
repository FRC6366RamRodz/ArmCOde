// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extend extends SubsystemBase {
  /** Creates a new Extend. */
  private final TalonFX extend;
  private final CANcoder _extendCANcoder;

  private final MotionMagicVoltage mmVolts = new MotionMagicVoltage(0).withSlot(0);

  public Extend() {
  extend = new TalonFX(20, "roborio");
  _extendCANcoder = new CANcoder(23,"roborio");
  TalonFXConfiguration cfg = new TalonFXConfiguration();
  Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
           FeedbackConfigs fdb = cfg.Feedback;
           cfg.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
           .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1)) // Take approximately 0.5 seconds to reach max vel
           // Take approximately 0.1 seconds to reach max accel 
           .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
           fdb.SensorToMechanismRatio = 1;
           fdb.RotorToSensorRatio = 36;
             fdb.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; //rezero CANcoder
             fdb.FeedbackRemoteSensorID = _extendCANcoder.getDeviceID();
            //  cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder; //rezero CANcoder
            //  cfg.Feedback.FeedbackRemoteSensorID = _pivotCANcoder.getDeviceID();
           extend.optimizeBusUtilization(50,20);
           extend.setPosition(_extendCANcoder.getAbsolutePosition().getValueAsDouble());
           extend.getConfigurator().apply(cfg);
  }

  public void moveExtend(double rotations){
    extend.setControl(mmVolts.withPosition(rotations).withSlot(0));
  }

  public boolean extendAtSetPoint(double atPosition){
    boolean positionTrueFalse;
    double difference = Math.abs(atPosition -  _extendCANcoder.getAbsolutePosition().getValueAsDouble()); //gets difference of the two
    positionTrueFalse = difference < 0.1; //sets the difference and how much it should be
       SmartDashboard.putBoolean("ExtendAtSetPoint", positionTrueFalse); //prints whether its true or false
    return positionTrueFalse; //returns true or false
    }
}
