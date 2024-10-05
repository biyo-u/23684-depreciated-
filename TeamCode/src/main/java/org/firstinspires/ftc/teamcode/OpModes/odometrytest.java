package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Disabled//(name = "FIRSTOdometryTest", group = "odotest")
public class odometrytest extends OpMode {

    /* This code will track the location of the odometry pods attached to certain motor ports on a predetermined path. This code does not let you directly choose how much the wheel turns.
    The next step for this is to replace the gamepad's preset distances with the Tech Titans' standard Mechanum Drivetrain Control Layout, but that's for when we turn the robot on.
    The coding crew should make a point of adding a ton of comments to our code, alongside explaining the general purpose of the code at the top. This will allow for both simplicity in deciphering
    the code, allowing for more seamless passage of information, but also make it faster to both edit and sort through different files. Perchance a changelog be added also? Emojis also work ❤️❤️ here!*/

    //create motor classes on benjamin (or the robot in question)
    private DcMotor leftFrontDrive; //0
    private DcMotor rightFrontDrive; //1
    private DcMotor leftBackDrive; //2
    private DcMotor rightBackDrive; //3

    //create variables that we can adjust
    double ticks = (336.00) * 10;
    /* encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double the ticks to double the rotations, or double the rotations to do the same */
    double newTarget; //for target position value

    double angle = ticks * 360; //tells you the angle of the motor, but can go over 360

    double angleNormalized = angle % 360; //tells you the angle of the motor, but remains between 0 and 360.

    //get Current Position of each encoder.
    int x1position = leftFrontDrive.getCurrentPosition();
    int x2position = rightFrontDrive.getCurrentPosition();
    int yposition = leftBackDrive.getCurrentPosition();

    //get revolution count of each encoder
    double x1revolutions = x1position/ticks;
    double x2revolutions = x2position/ticks;
    double yrevolutions = yposition/ticks;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public void init() {
        //initialising hardwaremaps for motors on driver hub
        leftBackDrive = hardwareMap.get(DcMotor.class, "rear_left");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rear_right");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");

        //print on driverhub that hardware is initalised
        telemetry.addData("Hardware", "initialised");

        //activate the encoders on the odometry pods
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //deactivate the wheel running without any odometry pod attached
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        //present position controls on gamepad 1
        if(gamepad1.a){
            odometry(1);
        }
        if(gamepad1.b){
            tracker();
        }

        // print out the current position of the encoders on driverhub (measured ticks)
        telemetry.addData("motor ticks: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("motor ticks: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("motor ticks: ", leftFrontDrive.getCurrentPosition());

        // print out the current angle of the encoders on driver hub (in degrees)
        telemetry.addData("Wheel Angle: ", angle);

        // print out the normalised angle of the motors on driver hub (in degrees)
        telemetry.addData("Wheel Angle (Normalised): ", angleNormalized);

        // print out the revolution count of the encoders on the driver hub
        telemetry.addData("Encoder X1 revolutions: ", x1revolutions);
        telemetry.addData("Encoder X2 revolutions: ", x2revolutions);
        telemetry.addData("Encoder Y revolutions: ", yrevolutions);

        //updates the telemetry
        telemetry.update();
    }

    public void odometry(int turnage){
        //set target value, which for this is whatever target it was set to be.
        newTarget = ticks/turnage;
        leftFrontDrive.setTargetPosition((int)newTarget);
        rightFrontDrive.setTargetPosition((int)newTarget);
        leftBackDrive.setTargetPosition((int)newTarget);

        //give power to the motors
        leftBackDrive.setPower(0.1);
        leftFrontDrive.setPower(0.1);
        rightBackDrive.setPower(0.1);
        rightFrontDrive.setPower(0.1);

        //run to position based on the odometry pod encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        //sets target value, which for this will be the home position of 0
        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);

        //give power to the motors
        leftBackDrive.setPower(0.1);
        leftFrontDrive.setPower(0.1);
        rightBackDrive.setPower(0.1);
        rightFrontDrive.setPower(0.1);

        //run to position based on the odometry pod encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}