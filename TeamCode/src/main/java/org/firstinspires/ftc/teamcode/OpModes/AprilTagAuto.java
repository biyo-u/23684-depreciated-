package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;
import android.util.Size;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

// April Tag Docs
// https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
// https://ftc-docs.firstinspires.org/apriltag-detection-values

@TeleOp(name = "AutoWithAprilTags", group = "Auto")
public class AprilTagAuto extends OpMode {

    // Vision portal
    private VisionPortal visionPortal;

    Robot robot;

    Telemetry telemetry;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, true);
        telemetry = robot.telemetry;
        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

    public void loop(){
        telemetryAprilTag();

        telemetry.update();

        visionPortal.resumeStreaming();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Call function to add april tag locations to Constants.aprilTagLocations
        Constants.addAprilTags();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(robot.webcam);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(Constants.Camera.width, Constants.Camera.height));

        builder.enableLiveView(Constants.developerMode);
        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether LiveView stops if no processors are enabled.
        builder.setAutoStopLiveView(!Constants.developerMode);

        // Set and enable the processor.
        builder.addProcessor(robot.aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(robot.aprilTag, true);

    }

    // Calculates and updates Odometry
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Vector2d detectedAprilTagLocation = Constants.aprilTagLocations.get(detection.id);

                if (detection.rawPose != null)   {
                    double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                    double robotX = detection.rawPose.x + Constants.Camera.x * Math.cos(heading);
                    double robotZ = detection.rawPose.z + Constants.Camera.z;
                    double robotY = -detection.rawPose.y + Constants.Camera.y * Math.sin(heading);

                    telemetry.addLine("April Tag Robot X: " + robotX);
                    telemetry.addLine("April Tag Robot Y: " + robotY);
                    telemetry.addLine("April Tag Robot Z: " + robotZ);

                    robot.odometry.updateOdometry(robotX, robotY);
                }

                telemetry.addLine(String.format("XY %6.1f %6.1f (inch)", detectedAprilTagLocation.x, detectedAprilTagLocation.y));
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


}   // end class
