package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OpModes.functions.Constants;

public class oop extends LinearOpMode {

    private DcMotor motor;

    int FORWARD_DRIVE_DISTANCE = 20;
    int step =1;
    int zone;
    boolean teampropdetected = true;
    boolean onright = false;
    boolean onleft = false;
    boolean onmiddle = true;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        int a = 1;
        if (a >= 0) {
        }

        //camera detect team prop, and will print out 1 for right, 2 for middle, 3 for left

        if (onright = true) {
            zone = 1;
        } else if (onmiddle = true) {
            zone = 2;
        } else if (onleft = true) {
            zone = 3;
        }

        switch (zone) {
            case 1:
                switch (step) {
                    case 11:
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motor.setTargetPosition((int)FORWARD_DRIVE_DISTANCE);
                        motor.setPower(0.5);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        step++;
                        break;
                }
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setTargetPosition((int)FORWARD_DRIVE_DISTANCE);
                motor.setPower(0.5);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case 2:
                //case 2 stuff ghoes here
            case 3:
        }
    }}