package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Auton {

    private DcMotor front_right;
    private DcMotor rear_right;
    private DcMotor front_left;
    private DcMotor rear_left;

    double ticks = 336.00;
    /* encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double the ticks to double the rotations, or double the rotations to do the same */
    double wheel_diameter = 3.7795; //in inches
    double power;

    public void __init__(DcMotor front_right, DcMotor front_left, DcMotor rear_left, DcMotor rear_right, double power) {
        // Initialise Motors
        this.front_left = front_left;
        this.front_right = front_right;
        this.rear_left = rear_left;
        this.rear_right = rear_right;
        this.power = power;

        // Reverse Motor
        this.rear_right.setDirection(DcMotor.Direction.REVERSE);

        // Setup and Reset Encoders
        this.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void move(double forward, double left){
        // X or Strafe is Front Right motor
        // Y or Forward is Front Left motor

        if (left != 0){
            // Calculate Strafe Target
            double xTarget = ticks/inchesToTicks(left);

            // Set Target position to Strafe motor
            front_right.setTargetPosition((int) Math.round(xTarget));

            // Handle Motor Directions for either Left or Right
            if (left > 0){
                // Rear Right must be the opposite of whatever value it needs to be
                rear_right.setDirection(DcMotor.Direction.FORWARD);
                // Everything else is fine
                rear_left.setDirection(DcMotor.Direction.FORWARD);
                front_left.setDirection(DcMotor.Direction.REVERSE);
                front_right.setDirection(DcMotor.Direction.FORWARD);
            }
            else if (left < 0){
                // Rear Right must be the opposite of whatever value it needs to be
                rear_right.setDirection(DcMotor.Direction.REVERSE);
                // Everything else is fine
                rear_left.setDirection(DcMotor.Direction.REVERSE);
                front_left.setDirection(DcMotor.Direction.FORWARD);
                front_right.setDirection(DcMotor.Direction.REVERSE);
            }

            // Set Power to All Motors
            front_right.setPower(power);
            front_left.setPower(power);
            rear_right.setPower(power);
            rear_left.setPower(power);

            // Set RunMode for all motors
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (front_right.isBusy()){
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

                // print out weather motors are busy
                telemetry.addData("Is Strafing: ", front_right.isBusy());
                telemetry.addData("Is going Forward: ", front_left.isBusy());

                // Send Data
                telemetry.update();
            }

            front_right.setPower(0);
            front_left.setPower(0);
            rear_right.setPower(0);
            rear_left.setPower(0);

            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setDirection(DcMotor.Direction.REVERSE);
            rear_left.setDirection(DcMotor.Direction.FORWARD);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_right.setDirection(DcMotor.Direction.FORWARD);
        }
        if (forward != 0){
            // Calculate Forward (or Backward if negative) Target
            double yTarget = ticks/inchesToTicks(forward);

            // Set Target position to Strafe motor
            front_left.setTargetPosition((int) Math.round(yTarget));

            // Rear Right must be the opposite of whatever value it needs to be
            rear_right.setDirection(DcMotor.Direction.REVERSE);
            // Everything else is fine
            rear_left.setDirection(DcMotor.Direction.FORWARD);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_right.setDirection(DcMotor.Direction.FORWARD);

            // Set Power to All Motors
            front_right.setPower(power);
            front_left.setPower(power);
            rear_right.setPower(power);
            rear_left.setPower(power);

            // Set RunMode for all motors
            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (front_left.isBusy()){
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

                // print out weather motors are busy
                telemetry.addData("Is Strafing: ", front_right.isBusy());
                telemetry.addData("Is going Forward: ", front_left.isBusy());

                // Send Data
                telemetry.update();
            }

            front_right.setPower(0);
            front_left.setPower(0);
            rear_right.setPower(0);
            rear_left.setPower(0);

            front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rear_right.setDirection(DcMotor.Direction.REVERSE);
            rear_left.setDirection(DcMotor.Direction.FORWARD);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_right.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void rotate(double degreesLeft){

    }

    private double inchesToTicks(double inches){
        return inches/(wheel_diameter * Math.PI * ticks);
    }
}
