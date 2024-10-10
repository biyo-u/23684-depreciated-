package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface IDrive {
    DcMotor front_left = null;
    DcMotor rear_left = null;
    DcMotor front_right = null;
    DcMotor rear_right = null;

    default void setFrontLeftSpeed(double speed){
        front_left.setPower(speed);
    }
    default void setFrontRightSpeed(double speed){
        front_right.setPower(speed);
    }
    default void setRearLeftSpeed(double speed){
        rear_left.setPower(speed);
    }
    default void setRearRightSpeed(double speed){
        rear_right.setPower(speed);
    }
}
