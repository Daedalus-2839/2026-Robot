package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class EstimatePose extends Command {

    private final CANdleSubsystem lights;
    private final CommandSwerveDrivetrain drivetrain;
    private final AimAndShoot.LimelightConfig[] cams;

    private int currentIndex = 0;
    private double lastTime = Timer.getFPGATimestamp(); // set to 0.0 is this doesn't work
    private final double interval = 0.5;
    private boolean lightsOn = false;

    public EstimatePose(
            CANdleSubsystem lights,
            CommandSwerveDrivetrain drivetrain,
            AimAndShoot.LimelightConfig[] cams) {

        this.lights = lights;
        this.drivetrain = drivetrain;
        this.cams = cams;

        addRequirements(lights);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
            if (now - lastTime >= interval) {
                for (AimAndShoot.LimelightConfig cam : cams) {
                    LimelightHelpers.setLEDMode_ForceOff(cam.name);
                }

                LimelightHelpers.setLEDMode_ForceOn(cams[currentIndex].name);
                LimelightHelpers.PoseEstimate ll = LimelightHelpers.getBotPoseEstimate_wpiBlue(cams[currentIndex].name);

                if (ll != null && ll.pose != null && ll.tagCount > 0) {
                drivetrain.addVisionMeasurement(ll.pose, ll.timestampSeconds);
                }

                if (lightsOn) {
                    lights.setColor(0, 0, 0);
                } else {
                    lights.setColor(0, 255, 0);
                }

                lightsOn = !lightsOn;

                currentIndex++;
                if (currentIndex >= cams.length) {
                    currentIndex = 0;
                }

                lastTime = now;
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (AimAndShoot.LimelightConfig cam : cams) {
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        lights.setColor(0, 0, 0);
        currentIndex = 0;
        lastTime = 0.0;
        lightsOn = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}