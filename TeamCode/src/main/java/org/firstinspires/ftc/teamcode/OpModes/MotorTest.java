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
public class MotorTest extends OpMode {
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
        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if(gamepad1.x){
            front_left.setPower(1);
        } else {
            front_left.setPower(0);
        }

        if(gamepad1.y){
            front_right.setPower(1);
        } else {
            front_right.setPower(0);
        }

        if(gamepad1.a){
            rear_left.setPower(1);
        } else {
            rear_left.setPower(0);
        }

        if(gamepad1.b){
            rear_right.setPower(1);
        } else {
            rear_right.setPower(0);
        }

        if(gamepad1.dpad_up){
            slide_left.setPower(1);
            slide_right.setPower(1);
        } else {
            slide_left.setPower(0);
            slide_right.setPower(0);
        }
    }
}