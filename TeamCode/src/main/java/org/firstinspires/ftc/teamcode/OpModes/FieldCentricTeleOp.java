package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Tele Op", group = "Tele Op")
public class FieldCentricTeleOp extends OpMode {
    DcMotor front_left;
    DcMotor rear_left;
    DcMotor front_right;
    DcMotor rear_right;
    DcMotor slide_left;
    DcMotor slide_right;
    IMU imu;
    double power;

    @Override
    public void init() {
        // Declare our motors
        // Make sure your ID's match your configuration
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        slide_left = hardwareMap.get(DcMotor.class, "slide_left");
        slide_right = hardwareMap.get(DcMotor.class, "slide_right");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
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
    }
    public void loop(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        if (gamepad1.right_bumper) {
            power = 1;
        } else {
            power = 1.5;
        }

        if(gamepad1.dpad_up){
            slide_left.setPower(1);
            slide_right.setPower(1);
        } else {
            slide_left.setPower(0);
            slide_right.setPower(0);
        }

        if(gamepad1.dpad_down){
            slide_left.setPower(-1);
            slide_right.setPower(-1);
        } else {
            slide_left.setPower(0);
            slide_right.setPower(0);
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = ((rotY + rotX + rx) / denominator) / power;
        double backLeftPower = ((rotY - rotX + rx) / denominator) / power;
        double frontRightPower = ((rotY - rotX - rx) / denominator) / power;
        double backRightPower = ((rotY + rotX - rx) / denominator) / power;

        front_left.setPower(frontLeftPower);
        rear_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        rear_right.setPower(backRightPower);
    }
}