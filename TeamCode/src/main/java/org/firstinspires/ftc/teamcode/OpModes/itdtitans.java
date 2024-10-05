package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "AUTOTITANS V10", group = "TECH TITANS 2025")
public class itdtitans extends OpMode {
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_front;
    private DcMotor left_back;

    double ticks = (336.00) * 10;

    double forward;
    double backward;
    double strafeleft;
    double straferight;
    double rotate;
    int target = 5;

    double angle = ticks * 360; //tells you the angle of the motor, but can go over 360

    double angleNormalized = angle % 360; //tells you the angle of the motor, but remains between 0 and 360.

    double wheel_diameter = 3.7795; //in inches
    int step =1;
    int pathway;
    boolean objectseen;
    boolean netzone;
    boolean observation;
    boolean chambers;

    @Override
    public void init() {
        //initialising hardwaremaps for motors on driver hub
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        //print on driverhub that hardware is initalised
        telemetry.addData("Hardware", "initialised");

        //activate the encoders on the odometry pods
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //deactivate the wheel running without any odometry pod attached
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (objectseen == true) {
            switch (pathway) {
                case 1:
                    movetochamber();
                case 2:
                    movetonetzone();
                case 3:
                    movetoobservationzone();
            }
        }
    }

    public void movetochamber() {
        //move code goes here
        if (chambers == true){
            right_front.setTargetPosition(target);
            if (right_front.getTargetPosition() < right_front.getCurrentPosition())
                right_front.setPower(0.7);
                right_back.setPower(0.7);
                left_back.setPower(0.7);
                left_front.setPower(0.7);
        }
        pathway = 2;
    }
    public void movetonetzone() {
        //move code goes here
        pathway = 3;
    }
    public void movetoobservationzone() {
        //move code goes here
        pathway = 4;
    }
}
