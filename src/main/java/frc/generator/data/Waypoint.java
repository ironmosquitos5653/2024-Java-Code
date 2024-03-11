package frc.generator.data;

public class Waypoint {
    
    private Point anchor;
    private Point prevControl;
    private Point nextControl;
    private boolean isLocked;
    private String linkedName;

    public Waypoint(Point anchor, Point prevControl, Point nextControl, boolean isLocked, String linkedName) {
        this.anchor = anchor;
        this.prevControl = prevControl;
        this.nextControl = nextControl;
        this.isLocked = isLocked;
        this.linkedName = linkedName;
    }
    public Point getAnchor() {
        return anchor;
    }

    public Point getPrevControl() {
        return prevControl;
    }

    public Point getNextControl() {
        return nextControl;
    }

    public boolean getIsLocked() {
        return isLocked;
    }

    public String getLinkedName() {
        return linkedName;
    }

    public void flipToRed() {
        anchor.flipToRed();
        prevControl.flipToRed();
        nextControl.flipToRed();
    }

    public static double flipAngle(double angle) {

        // top blue left 135
        // top red right = 45
        // 180 - blue = red

        // bottom blue left = -135
        // bottom right right -45
        // -180 - blue = red

        if (angle < 0) {
            return -180 - angle;
        }
        return 180 - angle;
    }
}
