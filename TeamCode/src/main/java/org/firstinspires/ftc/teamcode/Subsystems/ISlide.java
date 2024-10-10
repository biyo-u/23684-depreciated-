package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface ISlide {
    DcMotor slide_left = null;
    DcMotor slide_right = null;
    double left_extra_speed = 0;
    double right_extra_speed = 0;

    default void SlideMove(double speed, double left_extra_speed, double right_extra_speed){
        slide_left.setPower(speed + left_extra_speed);
        slide_right.setPower(speed + right_extra_speed);
    }

    default void SlideUp(double speed){
        SlideMove(speed, left_extra_speed, right_extra_speed);
    }

    default void SlideDown(double speed){
        SlideMove(-speed, left_extra_speed, right_extra_speed);
    }

    default void SlideStop(){
        SlideMove(0, 0, 0);
    }
}
