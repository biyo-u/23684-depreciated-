package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonTensorflowTest", group = "Autonomous")
public class AutonTensorflowTest extends OpMode {
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
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");

        telemetry.addData("Hardware", "initialised");

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

    }
}
