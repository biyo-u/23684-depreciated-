package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.*;

@Config
public class Constants {
    public static Map<Integer, Vector2d> aprilTagLocations = new HashMap<Integer, Vector2d>();
    // TODO: Disable dashboard for comps
    public static boolean DASHBOARD_ENABLED = true;
    public static boolean TELEMETRY_ENABLED = true;
    public static double aprilTagTrust = 0.25;
    public static double slideLeftExtra = 0;
    public static double slideRightExtra = 0;
    public static Map<Integer, Vector2d> aprilTagLocations = new HashMap<Integer, Vector2d>();

    public static void addAprilTags(){
        aprilTagLocations.put(11, new Vector2d(12, 12));
        aprilTagLocations.put(12, new Vector2d(12, 60));
        aprilTagLocations.put(13, new Vector2d(108, 12));
        aprilTagLocations.put(14, new Vector2d(108, 60));
        aprilTagLocations.put(15, new Vector2d(108, 108));
        aprilTagLocations.put(16, new Vector2d(12, 108));
    }

    // TODO: Measure camera position relative to center of robot
    public static class Camera {
        public static double x = 0;
        public static double y = 0;
        public static double z = 0;
        public static double yaw = 0;
        public static double pitch = 0;
        public static double roll = 0;
    }

}