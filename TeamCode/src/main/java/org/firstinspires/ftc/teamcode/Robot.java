package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Slide;

public class Robot {
    // Webcam for AprilTags
    public WebcamName webcam;
    // Library from GoBilda for their odometry computer
    public GoBildaPinpointDriver odo;
    // IMU is for direction, it is part of the control hub
    public IMU imu;
    // Power is for speed (percentage between 0 and 1)
    public double power;
    // Telemetry will be overridden with CAI's Telemetry, which sends telemetry to both the driver hub and FTC Dashboard
    public Telemetry telemetry;
    // Odometry will be initialized when AprilTags provide a reading
    // Requires an AprilTag to be in view when robot starts
    boolean isOdometryInitialized = false;

    public Slide slide;

    public Drive drive;

    // TODO: Only initialize required hardware depending on use case
    public Robot(HardwareMap hardwareMap, Telemetry ftcTelemetry) {
        // Uses CAI Telemetry to integrate with FTC Dashboard
        telemetry = new CAITelemetry(ftcTelemetry);
        // Gets the GoBuilda odometry computer
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        // All 4 motors
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        // Both slide motors, to move the slide up and down
        DcMotor slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        DcMotor slide_right = hardwareMap.get(DcMotor.class, "slide_right");
        webcam = hardwareMap.get(WebcamName.class, "Webcam");

        this.slide = new Slide(slide_left, slide_right);

        this.drive = new Drive(front_left, front_right, rear_left, rear_right);

        // Sets slide zero power mode to break so slide doesn't fall by itself
        // TODO: Add custom braking with higher power
        slide_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Sets Odometry offsets
        // TODO: Tune odometry offsets with our final robot
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.clearAll();
    }

    // TODO: Call updateOdometry from AprilTag Code
    public void updateOdometry(double x, double y){
        if (!isOdometryInitialized){
            // If there is no starting odometry position, set it using april tags and IMU
            odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, imu.getRobotYawPitchRollAngles().getYaw()));
            // then set it to initialize so it is not overridden, but set using a weight instead
            isOdometryInitialized = true;
        } else {
            // TODO: Find out how accurate april tags are
            double odoX = odo.getPosX();
            double odoY = odo.getPosY();

            // Sets odometry position with a weight value (Constants.aprilTagTrust is between 0 and 1)
            odo.setPosition(new Pose2D(DistanceUnit.INCH, (odoX * (1 - Constants.aprilTagTrust)) + (x * Constants.aprilTagTrust), (odoY * (1 - Constants.aprilTagTrust)) + (y * Constants.aprilTagTrust), AngleUnit.DEGREES, imu.getRobotYawPitchRollAngles().getYaw()));
        }
    }

    public void updateSpeed(double newPower) {
        power = newPower;
    }
}
