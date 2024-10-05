package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class autotitans2 extends OpMode {
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor left_front;
    private DcMotor left_back;

    double ticks = (336.00) * 10;

    double newTarget;
    double newerTarget;
    double newestTarget;

    double angle = ticks * 360; //tells you the angle of the motor, but can go over 360

    double angleNormalized = angle % 360; //tells you the angle of the motor, but remains between 0 and 360.

    double wheel_diameter = 3.7795; //in inches
    int step =1;
    int zone;
    boolean teampropdetected;
    boolean netzone = false;
    boolean observation = false;
    boolean chambers = true;
    
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
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //deactivate the wheel running without any odometry pod attached
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        //present position controls on gamepad 1
        if(gamepad1.a){
            target(1);
        } else if(gamepad1.b){
            tracker(true);
        }

        // print out the current position of the encoders on driverhub (measured ticks)
        telemetry.addData("motor ticks: ", left_back.getCurrentPosition());
        telemetry.addData("motor ticks: ", right_front.getCurrentPosition());
        telemetry.addData("motor ticks: ", left_front.getCurrentPosition());

        // print out the current angle of the encoders on driver hub (in degrees)
        // telemetry.addData("FL Wheel Angle: ", front_left.getCurrentPosition() * 360); //tells you the angle of the motor, but can go over 360
        // telemetry.addData("RL Wheel Angle: ", front_right.getCurrentPosition() * 360); //tells you the angle of the motor, but can go over 360

        // print out the normalised angle of the motors on driver hub (in degrees)
        telemetry.addData("Wheel Angle (Normalised) FL: ", (left_front.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
        telemetry.addData("Wheel Angle (Normalised) RL: ", (left_back.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.
        telemetry.addData("Wheel Angle (Normalised) FR: ", (right_front.getCurrentPosition() + 360) % 360); //tells you the angle of the motor, but remains between 0 and 360.

        // print out the revolution count of the encoders on the driver hub``````````````````````````````
        telemetry.addData("Encoder X1 revolutions: ", left_front.getCurrentPosition() / ticks);
        telemetry.addData("Encoder X2 revolutions: ", right_front.getCurrentPosition() / ticks);
        telemetry.addData("Encoder Y revolutions: ", left_back.getCurrentPosition() / ticks);

        // print out the ticks to inches count of the motor's movement amount
        telemetry.addData("Encoder X1 ticks to inches: ", left_front.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
        telemetry.addData("Encoder X2 ticks to inches: ", right_front.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));
        telemetry.addData("Encoder Y ticks to inches: ", left_back.getCurrentPosition() * (Math.PI /(wheel_diameter /4)));

        //updates the telemetry
        telemetry.update();
    }

    public void target(int turnage){

        //camera detect object, and will print out 1 for right, 2 for middle, 3 for left
        boolean teampropdetected = true;
        if (teampropdetected = true) {
            // code that confirms location
        }

        if (netzone = true) {
            zone = 1;
        } else if (chambers = true) {
            zone = 2;
        } else if (observation = true) {
            zone = 3;
        }
        
        switch (zone)  {
            case 1:
                switch (step) {
                    case 1:
                        //set target value, which for this is whatever target it was set to be.
                        newTarget = ticks/turnage;
                        left_front.setTargetPosition((int)newTarget);
                        right_front.setTargetPosition((int)newTarget);

                        //give power to the motors
                        left_back.setPower(0.3);
                        left_front.setPower(0.3);
                        right_back.setPower(0.3);
                        right_front.setPower(0.3);

                        //run to position based on the odometry pod encoders
                        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        if (right_front.getCurrentPosition() == newTarget) {
                            right_back.setPower(0);
                            left_back.setPower(0);
                        }
                        step = 2;
                    case 2:
                        //set target value, which for this is whatever target it was set to be.
                        newTarget = ticks/turnage;
                        left_front.setTargetPosition((int)newTarget);
                        right_front.setTargetPosition((int)newTarget);

                        //give power to the motors
                        left_back.setPower(-0.3);
                        left_front.setPower(-0.3);
                        right_back.setPower(-0.3);
                        right_front.setPower(-0.3);

                        //run to position based on the odometry pod encoders
                        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        if (right_front.getCurrentPosition() == newTarget) {
                            right_back.setPower(0);
                            left_back.setPower(0);
                        }
                        step = 3;
                }
            case 2:
                switch (step) {
                    case 4:
                        //yadda yadda
                    case 5:
                        //yadda yadda
                }
        }
    }
    
    public void tracker(boolean isseen){
        //RETURN TO ZERO
        //sets target value, which for this will be the home position of 0
        left_front.setTargetPosition(0);
        right_front.setTargetPosition(0);

        //give power to the motors
        left_back.setPower(0.3);
        left_front.setPower(0.3);
        right_back.setPower(0.3);
        right_front.setPower(0.3);

        //run to position based on the odometry pod encoders
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (right_front.getCurrentPosition() == 0) {
            right_back.setPower(0);
            left_back.setPower(0);
        }
    }
}
