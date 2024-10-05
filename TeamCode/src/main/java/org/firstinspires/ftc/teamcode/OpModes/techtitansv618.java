package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled//(name = "techtitansv618 (Blocks to Java)", group = "useabl opmodes")
public class techtitansv618 extends LinearOpMode {

    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor intake_liftAsDcMotor;
    private DcMotor lift;
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor intakeAsDcMotor;
    private Servo outtake_coverAsServo;
    private DcMotor hangAsDcMotor;
    private Servo drone_launcherAsServo;
    private Servo lift_armAsServo;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int upIntakePosition;
        int downIntakePosition;
        double armDeadband;
        int liftOff;
        int launch;
        boolean manualMode;
        int run;
        double power;

        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        intake_liftAsDcMotor = hardwareMap.get(DcMotor.class, "intake_liftAsDcMotor");
        lift = hardwareMap.get(DcMotor.class, "lift");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        intakeAsDcMotor = hardwareMap.get(DcMotor.class, "intakeAsDcMotor");
        // outtake_coverAsServo = hardwareMap.get(Servo.class, "outtake_coverAsServo");
        // hangAsDcMotor = hardwareMap.get(DcMotor.class, "hangAsDcMotor");
        // drone_launcherAsServo = hardwareMap.get(Servo.class, "drone_launcherAsServo");
        // lift_armAsServo = hardwareMap.get(Servo.class, "lift_armAsServo");

        // Put initialization blocks here.
        upIntakePosition = 0;
        downIntakePosition = 600;
        armDeadband = 0.03;
        liftOff = 1;
        launch = 0;
        manualMode = false;
        run = 0;
        // right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);
        intake_liftAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
        intake_liftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_liftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // liftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // liftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.\
                // Speed Controls
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    power = 1;
                } else {
                    power = 2.5;
                }
                // Driving Controls
                right_front.setPower((gamepad1.left_stick_x + gamepad1.right_stick_y + gamepad1.right_stick_x) / power);
                right_back.setPower((gamepad1.left_stick_x + (gamepad1.right_stick_y - gamepad1.right_stick_x)) / power);
                left_front.setPower((-gamepad1.left_stick_x + (gamepad1.right_stick_y - gamepad1.right_stick_x)) / power);
                left_back.setPower((-gamepad1.left_stick_x + gamepad1.right_stick_y + gamepad1.right_stick_x) / power);
                telemetry.update();
                // Intake Controls
                if (gamepad2.a) {
                    intakeAsDcMotor.setPower(-0.3);
                } else if (gamepad2.b) {
                    intakeAsDcMotor.setPower(0.3);
                } else {
                    intakeAsDcMotor.setPower(0);
                }
                // Intake Lift Controls
                if (gamepad2.dpad_left) {
                    intake_liftAsDcMotor.setPower(-0.7);
                } else if (gamepad2.dpad_right) {
                    intake_liftAsDcMotor.setPower(0.7);
                } else {
                    intake_liftAsDcMotor.setPower(0);
                }
                //LINEAR SLIDE(??)
                if (gamepad2.dpad_up) {
                    lift.setPower(-1);
                } else if (gamepad2.dpad_down) {
                    lift.setPower(1);
                } else {
                    lift.setPower(0);
                }
                /* // Cover Controls
                if (gamepad2.x) {
                    outtake_coverAsServo.setPosition(1);
                } else {
                    outtake_coverAsServo.setPosition(0);
                }
                // Hang Controls
                if (gamepad1.dpad_up) {
                    hangAsDcMotor.setPower(-1);
                } else if (gamepad1.dpad_down) {
                    hangAsDcMotor.setPower(1);
                } else {
                    hangAsDcMotor.setPower(0);
                }
                // Drone Launcher Controls
                if (gamepad2.y) {
                    drone_launcherAsServo.setPosition(1);
                } else {
                    drone_launcherAsServo.setPosition(0);
                }
                // Slide Arm Controls
                if (gamepad2.right_bumper) {
                    lift_armAsServo.setPosition(1);
                } else {
                    lift_armAsServo.setPosition(0);
                }*/
                // Linear Slide Controls
                    // UPDATE TELEMETRY
//                    telemetry.addData("Slide position", liftAsDcMotor.getCurrentPosition());
//                    telemetry.addData("Intake position", intake_liftAsDcMotor.getCurrentPosition());
//                    telemetry.update();
//                }
                    // hello world (real)
                }
            }
        }
    }
