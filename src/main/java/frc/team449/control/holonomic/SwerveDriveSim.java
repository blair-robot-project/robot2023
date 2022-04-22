package frc.team449.control.holonomic;

public class SwerveDriveSim extends SwerveDrive implements HolonomicDrive {
    private @NotNull Rotation2d heading;
    private final double simUpdateInterval = 0.02;

    public SwerveDriveSim(@NotNull heading, desiredSpeeds) {
        this.heading = heading;
    }

    @Override
    public double getHeading() {
        return this.heading;
    }

    @Override
    public void periodic() {
        this.heading = simUpdateInterval * desiredSpeeds.omegaRadiansPerSecond;
    }

}