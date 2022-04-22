package frc.team449.control.holonomic;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team449.system.AHRS;

public class SwerveDriveSim extends SwerveDrive implements HolonomicDrive {
    private double heading;
    private final double simUpdateInterval = 0.02;

    public SwerveDriveSim(AHRS ahrs, double maxLinearSpeed, double maxRotSpeed, Rotation2d heading, SwerveModule... modules) {
        super(ahrs, maxLinearSpeed, maxRotSpeed, modules);
    }

    @Override
    public Rotation2d getHeading() {
        return new Rotation2d(this.heading);
    }

    @Override
    public void periodic() {
        this.heading += simUpdateInterval * desiredSpeeds.omegaRadiansPerSecond;
    }

}