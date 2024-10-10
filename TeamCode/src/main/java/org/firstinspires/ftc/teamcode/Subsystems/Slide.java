package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Constants;

public class Slide implements ISlide {
    private DcMotor slide_left;
    private DcMotor slide_right;
    private double left_extra_speed;
    private double right_extra_speed;

    public Slide(DcMotor slide_left, DcMotor slide_right){
        this.slide_left = slide_left;
        this.slide_right = slide_right;
        this.left_extra_speed = Constants.slideLeftExtra;
        this.right_extra_speed = Constants.slideRightExtra;
        this.slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void SlideUp(double speed){
        SlideMove(speed, left_extra_speed, right_extra_speed);
    }

    public void SlideDown(double speed){
        SlideMove(-speed, left_extra_speed, right_extra_speed);
    }
}

