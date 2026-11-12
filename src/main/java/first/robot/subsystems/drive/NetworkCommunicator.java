package first.robot.subsystems.drive;

import org.wpilib.networktables.*;

public class NetworkCommunicator {
  private static NetworkCommunicator instance;
  private NetworkTableInstance ntInst;
  private StringArraySubscriber autoSub;
  private BooleanPublisher isAutoBooleanPublisher;

  // private HashMap<String, PathPlannerPath> paths;

  private NetworkCommunicator() {}

  public static NetworkCommunicator getInstance() {
    if (instance == null) {
      instance = new NetworkCommunicator();
    }
    return instance;
  }

  public void init() {
    ntInst = NetworkTableInstance.getDefault();
    NetworkTable table = ntInst.getTable("uidata");

    // Communication with touchscreen
    autoSub = table.getStringArrayTopic("autocommands").subscribe(new String[0]);
    isAutoBooleanPublisher = table.getBooleanTopic("isauto").publish();
    isAutoBooleanPublisher.set(true);
  }

  public void close() {
    autoSub.close();
  }

  public String[] getAutoCommands() {
    return autoSub.get();
  }

  public void setIsAuto(boolean isAuto) {
    isAutoBooleanPublisher.set(isAuto);
  }
}
