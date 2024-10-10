package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    // Setup All Variables
    public DcMotor front_left;
    public DcMotor rear_left;
    public DcMotor front_right;
    public DcMotor rear_right;
    public DcMotor slide_left;
    public DcMotor slide_right;
    public GoBildaPinpointDriver odo;
    public IMU imu;
    public double power;
    public Telemetry telemetry;
    boolean isOdometryInitialized = false;

    // TODO: Only initialize required hardware depending on use case
    public Robot(HardwareMap hardwareMap, Telemetry ftcTelemetry ) {
        telemetry = new CAITelemetry(ftcTelemetry);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        slide_right = hardwareMap.get(DcMotor.class, "slide_right");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_left.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_right.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);

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

        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.clearAll();
    }

    // TODO: Call updateOdometry from AprilTag Code
    public void updateOdometry(double x, double y){
        if (!isOdometryInitialized){
            odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, imu.getRobotYawPitchRollAngles().getYaw()));
            isOdometryInitialized = true;
        } else {
            // TODO: Find out how accurate april tags are
            double odoX = odo.getPosX();
            double odoY = odo.getPosY();

            odo.setPosition(new Pose2D(DistanceUnit.INCH, (odoX * (1 - Constants.aprilTagTrust)) + (x * Constants.aprilTagTrust), (odoY * (1 - Constants.aprilTagTrust)) + (y * Constants.aprilTagTrust), AngleUnit.DEGREES, imu.getRobotYawPitchRollAngles().getYaw()));
        }
    }
}
