package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;

public class ShooterLinearActuator extends SubsystemBase {

    private final TalonFX motor;
    private final double kP = 0.05;

    // Use meters
    private final double actuatorDistancePerRotation = 0.01;
    private final double minPosition = 0.0;
    private final double maxPosition = 0.2;

    private final DigitalInput forwardLimit;
    private final DigitalInput reverseLimit;

    public ShooterLinearActuator(int motorCANId, int forwardLimitChannel, int reverseLimitChannel) {
        motor = new TalonFX(motorCANId);
        UnpaidIntern.zeroEncoder(motor);

        forwardLimit = new DigitalInput(forwardLimitChannel);
        reverseLimit = new DigitalInput(reverseLimitChannel);
    }

    public void setPosition(double targetPosition) {

        targetPosition = Math.max(Math.min(targetPosition, maxPosition), minPosition);

        double currentRotations = motor.getRotorPosition().getValueAsDouble();
        double currentPosition = currentRotations * actuatorDistancePerRotation;

        double error = targetPosition - currentPosition;
        double output = error * kP;

        if ((forwardLimit.get() && output > 0) || (reverseLimit.get() && output < 0)) {
            output = 0.0;
        }

        output = Math.max(Math.min(output, 1.0), -1.0);

        motor.set(output);

        SmartDashboard.putNumber("Actuator Target Position", targetPosition);
        SmartDashboard.putNumber("Actuator Current Position", currentPosition);
        SmartDashboard.putNumber("Actuator Motor Output", output);
        SmartDashboard.putBoolean("Forward Limit Hit", forwardLimit.get());
        SmartDashboard.putBoolean("Reverse Limit Hit", reverseLimit.get());
    }

    public double getPosition() {
        return motor.getRotorPosition().getValueAsDouble() * actuatorDistancePerRotation;
    }

    public boolean isAtTarget(double targetPosition, double deadband) {
        double currentPosition = getPosition();
        double distance = Math.abs(currentPosition - targetPosition);
        return distance <= deadband;
    }

    public void stop() {
        motor.set(0.0);
    }
}