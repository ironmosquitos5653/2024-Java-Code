package frc.generator;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import frc.generator.data.AutoPath;
import frc.generator.data.EndState;
import frc.generator.data.GlobalConstraints;
import frc.generator.data.Point;
import frc.generator.data.Waypoint;

public class PathParser {

  private double previousAngle;
    
  public PathParser(double startAngle) {
    previousAngle = startAngle;
  }

  public double getPreviousAngle() {
    return previousAngle;
  }
  
  public void setPreviousAngle(double angle) {
    previousAngle = angle;
  }

  public AutoPath parsePath(String name) throws Exception {
      File pathFile = new File(AutoGenerator.PATHPLANNER_PATH + "/paths/" + name + ".path");
      JSONParser parser = new JSONParser();
      JSONObject json = (JSONObject) parser.parse(new FileReader(pathFile));

      double version = (double) json.get("version");
      boolean reversed = (boolean) json.get("reversed");
      String folder = (String) json.get("folder");
      Waypoint[] waypoints = parseWaypoints(json);
      GlobalConstraints globalConstraints = parseGlobalConstraints((JSONObject)json.get("globalConstraints"));
      EndState endState = parseEndState((JSONObject)json.get("goalEndState"));

      return new AutoPath(name, version, reversed, folder, waypoints, globalConstraints, endState);
  } 

  private Waypoint[] parseWaypoints(JSONObject json) {
    JSONArray jsonWaypoints = (JSONArray) json.get("waypoints");

    ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();

    for (int i = 0; i < jsonWaypoints.size(); i ++ ) {
      waypoints.add(parseWaypoint((JSONObject)jsonWaypoints.get(i)));
    }
    return waypoints.toArray(new Waypoint[waypoints.size()]);
    }
    private Waypoint parseWaypoint(JSONObject json) {
        Point anchor = parsePoint((JSONObject) json.get("anchor"));
        Point prevControl = parsePoint((JSONObject) json.get("anchor"));
        Point nextControl = parsePoint((JSONObject) json.get("anchor"));
        boolean isLocked = (boolean) json.get("isLocked");
        String linkedName = (String) json.get("linkedName");
        return new Waypoint(anchor, prevControl, nextControl, isLocked, linkedName);
    }

  private Point parsePoint(JSONObject json) {
    return new Point((double) json.get("x"), (double) json.get("y"));
  }

  private GlobalConstraints parseGlobalConstraints(JSONObject json) {
    return new GlobalConstraints(
            AutoParser.getDouble(json.get("maxVelocity")),
            AutoParser.getDouble(json.get("maxAcceleration")),
            AutoParser.getDouble(json.get("maxAngularVelocity")),
            AutoParser.getDouble(json.get("maxAngularAcceleration")));
  }

  private EndState parseEndState(JSONObject json) {
    return new EndState(AutoParser.getDouble(json.get("velocity")), AutoParser.getDouble(json.get("rotation")), (boolean)json.get("rotateFast"));
  }
}
