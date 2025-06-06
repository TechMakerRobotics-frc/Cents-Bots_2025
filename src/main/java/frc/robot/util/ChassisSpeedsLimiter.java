package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class ChassisSpeedsLimiter {
    private final double maxAcceleration = Constants.MAX_ACCELERATION; // m/s²
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    public ChassisSpeedsLimiter() {
    }

    public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds) {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt <= 0) {
            return lastSpeeds;
        }

        double vx = ramp(lastSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond, dt);
        double vy = ramp(lastSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond, dt);
        double omega = ramp(lastSpeeds.omegaRadiansPerSecond, targetSpeeds.omegaRadiansPerSecond, dt);

        lastSpeeds = new ChassisSpeeds(vx, vy, omega);
        return lastSpeeds;
    }

    private double ramp(double current, double target, double dt) {
        double delta = target - current;
        double maxDelta = maxAcceleration * dt;

        if (Math.abs(delta) > maxDelta) {
            return current + Math.copySign(maxDelta, delta);
        } else {
            return target;
        }
    }

    public void reset() {
        lastSpeeds = new ChassisSpeeds();
        lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
}
