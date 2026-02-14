package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;

public class Intake extends SubsystemBase {
    private final TalonFX motorA;
    private final TalonFX motorB;

    private static final double DEADBAND = 0;
    private static final double MAX_SPEED = 1;
    private static final double MIN_SPEED = -1;

    // Arm voltage constants
    private static final double ARM_HOLD = 0.1;
    private static final double ARM_MOVE = 0.2;
    private static final double ARM_UP_MOVE = 0.3;

    private boolean armUp = false;

    public Intake(int intakeMotorCANId, int armMotorCANId) {
        motorA = new TalonFX(intakeMotorCANId);
        motorB = new TalonFX(armMotorCANId);

        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
    }

    public void open(double speed) {
        timedArmDown();
        UnpaidIntern.setPercentWithDeadband(motorA, speed, DEADBAND);
    }

    public void close() {
        timedArmUp();
        UnpaidIntern.setPercentWithDeadband(motorA, 0, DEADBAND);
    }

    public void setIntake(double speed) {
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        UnpaidIntern.setPercentWithDeadband(motorA, speed, DEADBAND);
    }

    public void timedArmDown() {
        armUp = false;
        UnpaidIntern.setPercentWithDeadband(motorB, -ARM_MOVE, DEADBAND);
        Timer.delay(1.0);
        UnpaidIntern.stop(motorB);
    }

    public void timedArmUp() {
        armUp = true;
        UnpaidIntern.setPercentWithDeadband(motorB, ARM_UP_MOVE, DEADBAND);
        Timer.delay(1.0);
        UnpaidIntern.setPercentWithDeadband(motorB, ARM_HOLD, DEADBAND);
    }

    public void holdArm() {
        if (armUp) {
            UnpaidIntern.setPercentWithDeadband(motorB, ARM_HOLD, DEADBAND);
        } else {
            UnpaidIntern.stop(motorB);
        }
    }

    public void stop() {
        setIntake(0);
        UnpaidIntern.stop(motorB);
    }

    public void kill() {
        UnpaidIntern.killMotor(motorA);
        UnpaidIntern.killMotor(motorB);
    }

    public TalonFX getMotorA() { return motorA; }
    public TalonFX getMotorB() { return motorB; }
}