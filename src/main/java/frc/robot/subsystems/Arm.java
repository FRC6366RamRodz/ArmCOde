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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private final TalonFX pivot;
    private final CANcoder _pivotCANcoder;
    private final MotionMagicVoltage mmVolts = new MotionMagicVoltage(0).withSlot(0);

    public Arm(){
        pivot = new TalonFX(21, "roborio");
        _pivotCANcoder = new CANcoder(22, "roborio");
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
           fdb.SensorToMechanismRatio = 63;
             cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder; //rezero CANcoder
             cfg.Feedback.FeedbackRemoteSensorID = _pivotCANcoder.getDeviceID();
           pivot.optimizeBusUtilization(50,20);
           pivot.getConfigurator().apply(cfg);}

    public void moveArm(double rotations){
      pivot.setControl(mmVolts.withPosition(rotations).withSlot(0));
    }

}
