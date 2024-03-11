package frc.generator.data;

public class GlobalConstraints {
    private double maxVelocity;
    private double maxAcceleration;
    private double maxAngularVelocity;
    private double maxAngularAcceleration;

    public GlobalConstraints(double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public double getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }
}
