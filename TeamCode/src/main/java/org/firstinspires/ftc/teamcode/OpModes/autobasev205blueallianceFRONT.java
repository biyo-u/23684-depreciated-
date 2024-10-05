package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled //(name = "autobasev205blueallianceFRONT (Blocks to Java)")
public class autobasev205blueallianceFRONT extends LinearOpMode {

    private DcMotor right_backAsDcMotor;
    private DcMotor right_frontAsDcMotor;
    private DcMotor left_frontAsDcMotor;
    private DcMotor left_backAsDcMotor;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        right_backAsDcMotor = hardwareMap.get(DcMotor.class, "right_backAsDcMotor");
        right_frontAsDcMotor = hardwareMap.get(DcMotor.class, "right_frontAsDcMotor");
        left_frontAsDcMotor = hardwareMap.get(DcMotor.class, "left_frontAsDcMotor");
        left_backAsDcMotor = hardwareMap.get(DcMotor.class, "left_backAsDcMotor");

        // when on blue side
        // Put initialization blocks here.
        waitForStart();
        right_backAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
        right_frontAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
        left_frontAsDcMotor.setPower(0);
        left_backAsDcMotor.setPower(0);
        right_frontAsDcMotor.setPower(0);
        right_backAsDcMotor.setPower(0);
        sleep(20000);
        left_frontAsDcMotor.setPower(-0.5);
        left_backAsDcMotor.setPower(0.5);
        right_frontAsDcMotor.setPower(0.5);
        right_backAsDcMotor.setPower(-0.5);
        sleep(160);
        right_frontAsDcMotor.setPower(0);
        right_backAsDcMotor.setPower(0);
        left_frontAsDcMotor.setPower(0);
        left_backAsDcMotor.setPower(0);
        right_frontAsDcMotor.setPower(-0.5);
        right_backAsDcMotor.setPower(-0.5);
        left_frontAsDcMotor.setPower(-0.5);
        left_backAsDcMotor.setPower(-0.5);
        sleep(1000);
        right_frontAsDcMotor.setPower(0);
        right_frontAsDcMotor.setPower(0);
        left_frontAsDcMotor.setPower(0);
        left_backAsDcMotor.setPower(0);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
}
