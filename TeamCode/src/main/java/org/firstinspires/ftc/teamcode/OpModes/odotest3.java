package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/* This code will track the location of the odometry pods attached to certain motor ports on a custom path. This code lets you directly choose how much the wheel turns as it is run by
    the Tech Titans Standard Mechanum Drivetrain Control Layout (TTSMDCL for short). The coding crew should make a point of adding a ton of comments to our code, alongside explaining the general purpose of the code at the top.
    This will allow for both simplicity in deciphering the code, allowing for more seamless passage of information, but also make it faster to both edit and sort through different files.
    Perchance a changelog be added also? Emojis also work ❤️❤️ here! */

//ACTUALLY WORKSS

@TeleOp(name = "straferbase: odo test version", group = "odotest")
public class odotest3 extends LinearOpMode {

    private DcMotor front_right;
    private DcMotor rear_right;
    private DcMotor rear_left;
    private DcMotor front_left;

    double ticks = (336.00) * 10;
    /* encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double the ticks to double the rotations, or double the rotations to do the same */
    double wheel_diameter = 3.7795; //in inches

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double power;

        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");

        // Put initialization blocks here.
        front_right.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Speed Controls
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    power = 1;
                } else {
                    power = 2.5;
                }
                // Driving Controls
                front_left.setPower((gamepad1.left_stick_x + gamepad1.right_stick_y + gamepad1.right_stick_x) / power);
                rear_right.setPower((gamepad1.left_stick_x + (gamepad1.right_stick_y - gamepad1.right_stick_x)) / power);
                front_right.setPower((-gamepad1.left_stick_x + (gamepad1.right_stick_y - gamepad1.right_stick_x)) / power);
                rear_left.setPower((-gamepad1.left_stick_x + gamepad1.right_stick_y + gamepad1.right_stick_x) / power);

                // print out the current position of the encoders on driverhub (measured ticks)
                telemetry.addData("motor ticks: ", rear_left.getCurrentPosition());
                telemetry.addData("motor ticks: ", front_right.getCurrentPosition());
                telemetry.addData("motor ticks: ", front_left.getCurrentPosition());

                // print out the current angle of the encoders on driver hub (in degrees)
                telemetry.addData("FL Wheel Angle: ", front_left.getCurrentPosition() * 360); //tells you the angle of the motor, but can go over 360
                telemetry.addData("RL Wheel Angle: ", front_right.getCurrentPosition() * 360); //tells you the angle of the motor, but can go over 360

                // print out the normalised angle of the motors on driver hub (in degrees)
                telemetry.addData("Wheel Angle (Normalised) FL: ", (front_left.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
                telemetry.addData("Wheel Angle (Normalised) RL: ", (rear_left.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
                telemetry.addData("Wheel Angle (Normalised) FR: ", (front_right.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.

                // print out the revolution count of the encoders on the driver hub
                telemetry.addData("Encoder X1 revolutions: ", front_left.getCurrentPosition() / ticks);
                telemetry.addData("Encoder X2 revolutions: ", front_right.getCurrentPosition() / ticks);
                telemetry.addData("Encoder Y revolutions: ", rear_left.getCurrentPosition() / ticks);

                // print out the ticks to inches count of the motor's movement amount
                telemetry.addData("Encoder X1 ticks to inches: ", front_left.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
                telemetry.addData("Encoder X2 ticks to inches: ", front_right.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
                telemetry.addData("Encoder Y ticks to inches: ", rear_left.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));


                //updates the telemetry
                telemetry.update();
            }
        }
    }
}