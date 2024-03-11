package frc.generator;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.generator.data.Auto;
import frc.generator.data.AutoCommand;
import frc.generator.data.NamedAutoCommand;
import frc.generator.data.ParallelAutoCommand;
import frc.generator.data.PathAutoCommand;
import frc.generator.data.SequentialAutoCommand;
import frc.generator.data.WaitAutoCommand;

public class AutoParser {

  public static Auto parseAuto(File autoFile) throws Exception {
      JSONParser parser = new JSONParser();
      JSONObject json = (JSONObject) parser.parse(new FileReader(autoFile));

      String name = autoFile.getName().substring(0, autoFile.getName().length() - 5);
      double version = (double) json.get("version");
      Pose2d startingPose = parsePose((JSONObject)json.get("startingPose"));

      AutoCommand command = parseAutoCommand((JSONObject) json.get("command"));


      return new Auto(name, version, startingPose, command);
  } 
  
  private static AutoCommand parseAutoCommand(JSONObject json) {
    String commandType = (String) json.get("type");
    JSONObject data = (JSONObject) json.get("data");

    switch (commandType) {
      case "named":
        return parseNamedAuto(data);
      case "sequential":
        return parseSequentialAuto(data);

      case "parallel":
        return parseParallelAuto(data);
        
      case "path":
        return parsePathAuto(data);

      case "wait":
        return parseWaitAuto(data);
    }

    return null;
  }

  private static Pose2d parsePose(JSONObject json) {
    JSONObject position = (JSONObject)json.get("position");
    double x = (double) position.get("x");
    double y = (double) position.get("y");
    double rotation = getDouble(json.get("rotation"));

    return new Pose2d(new Translation2d(x, y), new Rotation2d(Units.degreesToRadians(rotation)));
  }

  private static NamedAutoCommand parseNamedAuto(JSONObject json) {
    return new NamedAutoCommand((String) json.get("name"));
  }

  private static WaitAutoCommand parseWaitAuto(JSONObject json) {
    return new WaitAutoCommand(getDouble(json.get("waitTime")));
  }

  private static PathAutoCommand parsePathAuto(JSONObject json) {
    return new PathAutoCommand((String) json.get("pathName"));
  }

  private static ParallelAutoCommand parseParallelAuto(JSONObject json) {
    AutoCommand[] commands = parseCommands(json);
    return new ParallelAutoCommand(commands);
  }

  private static SequentialAutoCommand parseSequentialAuto(JSONObject json) {
    AutoCommand[] commands = parseCommands(json);
    return new SequentialAutoCommand(commands);
  }

  private static AutoCommand[] parseCommands(JSONObject json) {
    JSONArray jsonCommands = (JSONArray) json.get("commands");

    ArrayList<AutoCommand> commands = new ArrayList<AutoCommand>();

    for (int i = 0; i < jsonCommands.size(); i ++ ) {
      commands.add(parseAutoCommand((JSONObject)jsonCommands.get(i)));
    }
    return commands.toArray(new AutoCommand[commands.size()]);
  }

  public static double getDouble(Object x) {
    return Double.valueOf(x.toString());
  }
}
