package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;
import frc.robot.subsystems.WebServer;

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {
    private final TalonFX motorA;
    private final TalonFX motorB;
    private final TalonFX motorC;

    private static final double DEADBAND = 0;
    private static final double MAX_SPEED = 1;
    private static final double MIN_SPEED = -1;
    private static final double BELT_DELAY_SEC = 1.0;

    private final Timer beltTimer = new Timer();
    private boolean beltStarted = false;
    private double currentSpeed = 0;

    public Shooter(int beltMotorACANId, int shooterMotorBCANId, int shooterMotorCCANId) {
        motorA = new TalonFX(beltMotorACANId);
        motorB = new TalonFX(shooterMotorBCANId);
        motorC = new TalonFX(shooterMotorCCANId);

        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
        UnpaidIntern.zeroEncoder(motorC);
    }

    private double clamp(double speed) {
        if (speed > MAX_SPEED) return MAX_SPEED;
        if (speed < MIN_SPEED) return MIN_SPEED;
        return speed;
    }

    public void set(double speed) {
        speed = clamp(speed);
        currentSpeed = speed;

        WebServer.putNumber("TargetSpeed", speed*100);
        WebServer.putNumber("BeltSpeed", motorA.getVelocity().getValueAsDouble());
        WebServer.putNumber("ShooterBSpeed", motorB.getVelocity().getValueAsDouble());
        WebServer.putNumber("ShooterCSpeed", motorC.getVelocity().getValueAsDouble());

        UnpaidIntern.setPercentWithDeadband(motorB, speed, DEADBAND);
        UnpaidIntern.setPercentWithDeadband(motorC, speed, DEADBAND);

        if (!beltTimer.isRunning()) {
            beltTimer.reset();
            beltTimer.start();
            beltStarted = false;
        }

        if (beltTimer.hasElapsed(BELT_DELAY_SEC)) {
            UnpaidIntern.setPercentWithDeadband(motorA, speed, DEADBAND);
            beltStarted = true;
        }
    }

    @Override
    public void periodic() {
        if (beltStarted) {
            UnpaidIntern.setPercentWithDeadband(motorA, currentSpeed, DEADBAND);
        }
    }

    public void stop() {
        beltTimer.stop();
        beltTimer.reset();
        beltStarted = false;

        UnpaidIntern.stop(motorA);
        UnpaidIntern.stop(motorB);
        UnpaidIntern.stop(motorC);
    }

    public void kill() {
        beltTimer.stop();
        beltTimer.reset();
        beltStarted = false;

        UnpaidIntern.killMotor(motorA);
        UnpaidIntern.killMotor(motorB);
        UnpaidIntern.killMotor(motorC);
    }
}