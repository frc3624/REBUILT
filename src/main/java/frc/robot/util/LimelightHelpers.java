package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {

    private static NetworkTable table(String name) {
        return NetworkTableInstance.getDefault().getTable(name);
    }

    public static boolean hasTarget(String name) {
        return table(name).getEntry("tv").getDouble(0) == 1;
    }

    public static double getTX(String name) {
        return table(name).getEntry("tx").getDouble(0);
    }

    public static double getTA(String name) {
        return table(name).getEntry("ta").getDouble(0);
    }

    public static double getAprilTagID(String name) {
        return table(name).getEntry("tid").getDouble(-1);
    }
}
