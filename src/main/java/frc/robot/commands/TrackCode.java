package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TrackCode extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private static final double TARGET_DISTANCE = 1.2;

    private static final double kP_DIST = 1.1;
    private static final double kP_STRAFE = 1.0;
    private static final double kP_ROT = 0.01;

    private static final double MAX_VEL = 2.0;
    private static final double MAX_OMEGA = 2.5;

    public TrackCode(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        LimelightHelpers.setLEDMode_ForceOn("limelight-front");

        if (!LimelightHelpers.getTV("limelight-front")) {
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double tx = LimelightHelpers.getTX("limelight-front");

        double forwardError = LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[0] - TARGET_DISTANCE;
        double strafeError = LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[1];
        double angleError = Math.toRadians(tx);

        double vx = forwardError * kP_DIST;
        double vy = strafeError * kP_STRAFE;
        double omega = angleError * kP_ROT;

        vx = Math.max(-MAX_VEL, Math.min(MAX_VEL, vx));
        vy = Math.max(-MAX_VEL, Math.min(MAX_VEL, vy));
        omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, omega));

        drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        LimelightHelpers.setLEDMode_ForceOff("limelight-front");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}