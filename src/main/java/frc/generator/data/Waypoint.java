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
}
