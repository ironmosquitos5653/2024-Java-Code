package frc.generator.data;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.generator.AutoGenerator;

public class Point {
    private double x;
    private double y;


    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void flipToRed() {
        x = AutoGenerator.getAprilTagFieldLayout().getFieldLength() - x;
    }
}