package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Mecanum Autonomous", group = "Tech Titans Autonomous")
public class MecanumAutonomous extends OpMode {
    private DcMotor front_right;
    private DcMotor rear_right;
    private DcMotor front_left;
    private DcMotor rear_left;
    private IMU imu;

    double ticks = 336.00;
    /* encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double the ticks to double the rotations, or double the rotations to do the same */
    double wheel_diameter = 3.7795; //in inches
    double power = 0.2;
    Auton utils;

    @Override
    public void init() {
        // Initialise Motors and IMU
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        imu = hardwareMap.get(IMU.class, "imu");

        utils = new Auton();
        utils.__init__(front_right, front_left, rear_left, rear_right, power);

        // Reverse Motor
        rear_right.setDirection(DcMotor.Direction.REVERSE);

        // Setup and Reset Encoders
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // setup IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // init
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        // print out the current position of the encoders on driver hub (measured ticks)
        telemetry.addData("pod X ticks: ", front_right.getCurrentPosition());
        telemetry.addData("pod Y ticks: ", front_left.getCurrentPosition());

        // print out the normalised angle of the motors on driver hub (in degrees)
        telemetry.addData("Wheel Angle (Normalised) FL: ", (front_left.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
        telemetry.addData("Wheel Angle (Normalised) FR: ", (front_right.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.

        // print out the revolution count of the encoders on the driver hub
        telemetry.addData("Encoder Y revolutions: ", front_left.getCurrentPosition() / ticks);
        telemetry.addData("Encoder X revolutions: ", front_right.getCurrentPosition() / ticks);

        // print out the ticks to inches count of the motor's movement amount
        telemetry.addData("Encoder Y ticks to inches: ", front_left.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
        telemetry.addData("Encoder X ticks to inches: ", front_right.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));

        // print out the yaw of IMU for spinnnn
        telemetry.addData("benny's yaw pitch roll value:",imu.getRobotYawPitchRollAngles());

    }
}
