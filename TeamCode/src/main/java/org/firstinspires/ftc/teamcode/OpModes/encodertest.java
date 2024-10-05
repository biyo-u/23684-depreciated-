package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled//(name = "EncoderTrial ")
public class encodertest extends OpMode {
    DcMotor motor;
    // encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    // ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    // ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double ticks = (146.44) * 10;
    double newTarget;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "intake");
        telemetry.addData("Hardware", "initialised");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            encodertrial(1);
        }
        telemetry.addData("motor ticks: ", motor.getCurrentPosition());
        if(gamepad1.b){
            tracker();
        }
        if(gamepad1.y){
            ;
        }

    }
    public void encodertrial(int turnage){
        newTarget = ticks/turnage;
        motor.setTargetPosition((int)newTarget);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void tracker(){
        motor.setTargetPosition(0);
        motor.setPower(0.1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
