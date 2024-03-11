package frc.generator.data;

import java.util.ArrayList;

public class AutoPath {
    private String name;
    private double version;
    private Waypoint[] waypoints;
    private GlobalConstraints globalConstraints;
    private EndState endState;
    private boolean reversed;
    private String folder;

    public AutoPath(String name, double version, boolean reversed, String folder, Waypoint[] waypoints, GlobalConstraints globalConstraints, EndState endState) {
        this.name = name;
        this.version = version;
        this.waypoints = waypoints;
        this.globalConstraints = globalConstraints;
        this.endState = endState;
        this.reversed = reversed;
        this.folder = folder;
    }

    public String getName() {
        return name;
    }

    public double getVersion() {
        return version;
    }

    public Waypoint[] getWaypoints() {
        return waypoints;
    }

    public GlobalConstraints getGlobalConstraints() {
        return globalConstraints;
    }

    public EndState getEndState() {
        return endState;
    }

    public boolean getReversed() {
        return reversed;
    }

    public String getFolder() {
        return folder;
    }

    public Waypoint getStartPoint() {
        return waypoints[0];
    }

    public Waypoint getEndPoint() {
        return waypoints[waypoints.length - 1];
    }

    public Waypoint[] getInteriorWaypoints() {
        ArrayList<Waypoint> interior = new ArrayList<Waypoint>();
        for (int i = 1; i < waypoints.length - 1; i++) {
            interior.add(waypoints[i]);
        }
        return interior.toArray(new Waypoint[interior.size()]);
    
    }

}
