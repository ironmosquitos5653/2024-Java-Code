package frc.generator.data;

public class EndState {
    private double velocity;
    private double rotation;
    private boolean rotateFast;

    public EndState(double velocity, double rotation, boolean rotateFast) {
        this.velocity = velocity;
        this.rotation = rotation;
        this.rotateFast = rotateFast;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean getRotateFast() {
        return rotateFast;
    }
}
