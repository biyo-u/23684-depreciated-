package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.OpModes.functions.Constants;

/*
This code tries a modular approach to what CAI implemented for their telemetry, and what we used last year. The concept is that using the encodertest.java design, we can create separate movements

Various Notes:
    - RUN_WITHOUT_ENCODER just sends a power to the motors which may vary in speed depending on battery, friction and other variables. RUN_USING_ENCODER and RUN_TO_POSITION both use the MOTOR encoders, not the ODOMETRY encoders
 */
@TeleOp(name = "drivetrainODOconcept")
public class customdesignautoidea extends OpMode {

    private DcMotor front_right;
    private DcMotor rear_right;
    private DcMotor rear_left;
    private DcMotor front_left;
    double ticks = (336.00) * 2;
    /* encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double the ticks to double the rotations, or double the rotations to do the same */
    double wheel_diameter = 3.7795; //in inches
    double newTarget;
    double newerTarget;
    double newestTarget;

    //CONTROL HUB ORIENTATION
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    @Override
    public void init() {
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        front_right.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.REVERSE);

        //print on driverhub that hardware is initalised
        telemetry.addData("Hardware", "initialised");

        //activate the encoders on the odometry pods
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //deactivate the wheel running without any odometry pod attached
        rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // print out the current position of the encoders on driverhub (measured ticks)
        telemetry.addData("motor ticks: ", rear_left.getCurrentPosition());
        telemetry.addData("motor ticks: ", front_right.getCurrentPosition());
        telemetry.addData("motor ticks: ", front_left.getCurrentPosition());

        // print out the current angle of the encoders on driver hub (in degrees)
        // telemetry.addData("FL Wheel Angle: ", front_left.getCurrentPosition() * 360); //tells you the angle of the motor, but can go over 360
        // telemetry.addData("RL Wheel Angle: ", front_right.getCurrentPosition() * 360); //tells you the angle of the motor, but can go over 360

        // print out the normalised angle of the motors on driver hub (in degrees)
        telemetry.addData("Wheel Angle (Normalised) FL: ", (front_left.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
        telemetry.addData("Wheel Angle (Normalised) RL: ", (rear_left.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
        telemetry.addData("Wheel Angle (Normalised) FR: ", (front_right.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.

        // print out the revolution count of the encoders on the driver hub``````````````````````````````
        telemetry.addData("Encoder X1 revolutions: ", front_left.getCurrentPosition() / ticks);
        telemetry.addData("Encoder X2 revolutions: ", front_right.getCurrentPosition() / ticks);
        telemetry.addData("Encoder Y revolutions: ", rear_left.getCurrentPosition() / ticks);

        // print out the ticks to inches count of the motor's movement amount
        telemetry.addData("Encoder X1 ticks to inches: ", front_left.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
        telemetry.addData("Encoder X2 ticks to inches: ", front_right.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
        telemetry.addData("Encoder Y ticks to inches: ", rear_left.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
        if(gamepad1.a){
            target(1);
        } else if(gamepad1.x){
            target2(1);
        } else if(gamepad1.y)
            target3(1);
        if(gamepad1.b){
            tracker();
        }
        // updates the telemetry on Driver Hub
        telemetry.update();
    }
    public void target(int turnage){
        //MOVE FORWARRD TWO ROTATIONS
        //set target value, which for this is whatever target it was set to be.
        newTarget = ticks/turnage;
        front_left.setTargetPosition((int)newTarget);
        front_right.setTargetPosition((int)newTarget);

        //give power to the motors
        rear_left.setPower(0.3);
        front_left.setPower(0.3);
        rear_right.setPower(0.3);
        front_right.setPower(0.3);

        //run to position based on the odometry pod encoders
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (front_right.getCurrentPosition() == newTarget) {
            rear_right.setPower(0);
            rear_left.setPower(0);
        }
    }
    public void tracker(){
        //RETURN TO ZERO
        //sets target value, which for this will be the home position of 0
        front_left.setTargetPosition(0);
        front_right.setTargetPosition(0);

        //give power to the motors
        rear_left.setPower(0.3);
        front_left.setPower(0.3);
        rear_right.setPower(0.3);
        front_right.setPower(0.3);

        //run to position based on the odometry pod encoders
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (front_right.getCurrentPosition() == 0) {
            rear_right.setPower(0);
            rear_left.setPower(0);
        }
    }
    public void target2(int turnage) {
        //MOVE BACKWARD TWO ROTATIONS
        //set target value, which for this is whatever target it was set to be.
        newerTarget = ticks/turnage;
        front_left.setTargetPosition((int)newerTarget);
        front_right.setTargetPosition((int)newerTarget);

        //give power to the motors
        rear_left.setPower(-0.3);
        front_left.setPower(-0.3);
        rear_right.setPower(-0.3);
        front_right.setPower(-0.3);

        //run to position based on the odometry pod encoders
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (front_right.getCurrentPosition() == newerTarget) {
            rear_right.setPower(0);
            rear_left.setPower(0);
        }
    }
    public void target3(int turnage) {
        //STAFE SIDEWAYS TWO ROTATIONS
        //set target value, which for this is whatever target it was set to be.
        newestTarget = ticks/turnage;
        rear_left.setTargetPosition((int)newestTarget);

        //give power to the motors (STRAFE MEANS OPPOSITE CORNERS GO IN THE SAME DIRECTION)
        rear_left.setPower(0.3);
        front_left.setPower(-0.3);
        rear_right.setPower(-0.3);
        front_right.setPower(0.3);

        //run to position based on the odometry pod encoders
        rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (rear_left.getCurrentPosition() == newestTarget) {
            front_left.setPower(0);
            rear_right.setPower(0);
            front_right.setPower(0);
        }
    }
}
