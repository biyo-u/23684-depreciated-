package org.firstinspires.ftc.teamcode.OpModes.functions;

// hardware imports
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// program imports
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Version 7.0.0 of the Tech Titans TeleOp Drivetrain
#.#.# = major release.minor release.tiny change/bugfix */
@Disabled // (name = "Tech Titans V7.0.0 PRE-ALPHA")
public class techtitansv7 {

    // Drivebase Motors
    private final Motor front_right; //0
    private final Motor front_left; //1
    private final Motor rear_left; //2
    private final Motor rear_right; //3
    MechanumDrive drivebase;

    // Odometry and Variables
    private Motor xRPea; //shoudld've asked for what xPea and yPea are oops
    private Motor xLPea; // also need to figure out why i cant make 'private final Motor xRPea/xLPea/yPea' methods...
    private Motor yPea;
    private PIDController headingControl = null; // heading control on PID
    private PIDController xLControl = null; // Left X Encoder
    private PIDController xRControl = null; // Right X Encoder
    private PIDController yControl = null; // Y Encoder

    // Define IMU
    private IMU imu; // need to figure how why 'private final IMU imu;' does not function...

    // Auto Alignment Direction Variables
    public double forward = 0; // north
    public double back = 180; // south
    public double headingCorrection = 0;
    public double right = -90; // east
    public double left = 90; //west
    public double headingSetPoint = forward;
    public double heading; // Current Heading

    // 'false' if in TELEOP, 'true' if in AUTO
    private boolean AUTOEnabled = false;

    HardwareMap hwMap; // HardwareMap Variable

    // Speed
    private double currentSpeed = 0; // Current Speed of Robot
    double xRSpeed = 0; // Speed
    double xLSpeed = 0;
    double ySpeed = 0;

    // Movement Targets
    private double currentXLTarget = 0; // Target X Location
    private double currentXRTarget = 0; // Target X Location
    private double currentYTarget = 0; // Target Y Location

    public techtitansv7(HardwareMap hwMap) {
        this.hwMap = hwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        front_right = new Motor(hwMap, "front_right"); // PORT 0
        front_left = new Motor(hwMap, "front_right"); // PORT 1
        rear_left = new Motor(hwMap, "front_right"); // PORT 2
        rear_right = new Motor(hwMap, "front_right"); // PORT 3

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        drivebase = new MechanumDrive(); // 'driveBase = new MecanumDrive(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive);' did not work, will need to work out

    }

    public void init() {
        headingControl = new PIDController(0.005, 0.002, 0.0005);
        headingControl.setTolerance(10);// was 1 ..cbw
        xRControl = new PIDController(0.07, 0.2, 0.01); // need to find the correct kp, ki, and kd ratios for our drivetrain + odometry pods set up
        xRControl = new PIDController(0.07, 0.2, 0.01);
        yControl = new PIDController(0.07, 0.2, 0.01);

        rear_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); // redundant as default is brake mode
        rear_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        // this will make getDistance in inches, not ticks (NOTE: their TICKS_TO_INCHES constant =/= our TICKS_TO_INCHES constant!!! find correct value when possible!!)
        xRPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
        xRPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
        yPea.setDistancePerPulse(Constants.TICKS_TO_INCHES);
//        imu.resetYaw(); (was originally commented out)
    }

    public void start() {
        //resets encoders
        xRPea.resetEncoder();
        xLPea.resetEncoder();
        yPea.resetEncoder();
    }

    public void loop() {
        heading = imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit());

        // Because it's a double, can't check for exactly 180, so we check if it's almost 180 in either direction.
        if (Math.abs(Math.abs(headingSetPoint) - 180.0) < Constants.HEADING_ERROR) { //this is how the heading lock functions (FIND CORRECT VALUE FOR OUR ROBOT)

            // "south" is special because it's around the 180/-180 toggle-point
            // Change set-point between 180/-180 depending on which is closer.
            if (heading < 0.0) {
                headingSetPoint = -180;
            } else {
                headingSetPoint = 180;
            }
        }
        // PID controller for heading
        headingControl.setSetPoint(headingSetPoint);
        headingCorrection = -headingControl.calculate(heading);

        if(AUTOEnabled && !atTarget()) { //limits auto speed
            double xRTargetSpeed = -xRControl.calculate(getXRPosition());
            double xLTargetSpeed = -xLControl.calculate(getXLPosition());
            double yTargetSpeed = -yControl.calculate(getYPosition());
            double targetSpeed = currentSpeed;// Math.sqrt(yTargetSpe
            // ed * yTargetSpeed + xTargetSpeed * xTargetSpeed);
            xRSpeed = xRTargetSpeed * targetSpeed;
            ySpeed = yTargetSpeed * targetSpeed;
            xLSpeed = xLTargetSpeed * targetSpeed;
        }

        techtitansv7.driveFieldCentric(
                xRSpeed,
                xLSpeed,
                ySpeed,
                headingCorrection,
                heading,
                false);
    }
    public boolean atTarget() { // Pythagorean theorem to get the distance from target as a number. //check if it's close to auto
        double xError = getXRPosition() - currentXRTarget;
        double XLError = getXLPosition() - currentXLTarget;
        double yError = getYPosition() - currentYTarget;

        //DRIVE_PID_ERROR is a constant of how close it is to the destination on auto, always returns true if in a TeleOp program. (find number for OUR robot)
        return !AUTOEnabled;
        // return !AUTOEnabled || Math.sqrt(xError * xError + yError * yError) < Constants.DRIVE_PID_ERROR; //this needs to be adapted to 3 wheel odo using IIRC diff between xL and xR encoders
    }
    private static void driveFieldCentric(double xRSpeed, double xSpeed, double ySpeed, double headingCorrection, double heading, boolean b) {
    }
    public void drive(double forwardSpeed,  double strafeSpeed) { //for TeleOp, gives stick values and assigns to X and Y speeds, how PID gets the speeds it needs
        // tell ftclib its inputs  strafeSpeed,forwardSpeed,turn,heading
        // turn and heading are managed in loop with the heading control PID
        xRSpeed = forwardSpeed;
        xLSpeed = forwardSpeed;
        ySpeed = strafeSpeed;
    }
    //THESE THREE DOUBLES ARE NOT CORRECT, AND WILL NEED TO BE CHANGED TO WORK FOR 3 WHEEL ODOMETRY + X IS THE SIDEWAYS ODO, Y IS THE FORWARD ONE
    public double getXRPosition() { // Convert xRPod into actual direction based on current heading //checks which is x and which is y depending on rotation of bot (if at 180, gets Y, and -Y at -180. then it gets the X value at 90 and -X at -90
        double xRPos = 0.0;
        if(Math.abs(headingSetPoint - Constants.back) < Constants.HEADING_ERROR) xRPos = -xRPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.left) < Constants.HEADING_ERROR) xRPos = yPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.forward) < Constants.HEADING_ERROR) xRPos = xRPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.right) < Constants.HEADING_ERROR) xRPos = -yPea.getDistance();
        return xRPos;
    }
    public double getXLPosition() { // Convert xLPod into actual direction based on current heading //checks which is x and which is y depending on rotation of bot (if at 180, gets Y, and -Y at -180. then it gets the X value at 90 and -X at -90
        double xLPos = 0.0;
        if(Math.abs(headingSetPoint - Constants.back) < Constants.HEADING_ERROR) xLPos = -xLPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.left) < Constants.HEADING_ERROR) xLPos = yPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.forward) < Constants.HEADING_ERROR) xLPos = xLPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.right) < Constants.HEADING_ERROR) xLPos = -yPea.getDistance();
        return xLPos;
    }
    public double getYPosition() { // Convert yPod into actual direction based on current heading
        double yPos = 0.0;
        if(Math.abs(headingSetPoint - Constants.back) < Constants.HEADING_ERROR) yPos = yPea.getDistance(); //gets y value from odo pod,
        else if(Math.abs(headingSetPoint - Constants.left) < Constants.HEADING_ERROR) yPos = xLPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.forward) < Constants.HEADING_ERROR) yPos = -yPea.getDistance();
        else if(Math.abs(headingSetPoint - Constants.right) < Constants.HEADING_ERROR) yPos = -xRPea.getDistance();
        return yPos;
    }
    // done outside drivetrain so auto can also call it, so other things can reset the encoder
    public void resetXREncoder() { //would be doubled for three wheel odo
        xRPea.resetEncoder();
    }
    public void resetXLEncoder() { //would be doubled for three wheel odo
        xLPea.resetEncoder();
    }
    public void resetYEncoder() {
        yPea.resetEncoder();
    }
    public void resetIMU() {
        imu.resetYaw();
    }
    public void stop() { //stops robot motors, stops drivebase, turns auto off. when not used, it doesnt break the waypoint, and misses the desired point
        xLSpeed = 0.0;
        xRSpeed = 0.0;
        ySpeed = 0.0;
      //  techtitansv7.stop();
        AUTOEnabled = false;
    }
    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("actual heading:", "%5.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("saved heading:", "%5.2f", heading);
        telemetry.addData("heading Target:", headingSetPoint);
        telemetry.addData("heading correction:", headingCorrection);
        telemetry.addData("XR Distance inches:", "%5.2f", getXRPosition());
        telemetry.addData("XL Distance inches:", "%5.2f", getXLPosition());
        telemetry.addData("Y Distance inches:", "%5.2f", getYPosition());
        telemetry.addData("Auto DriveTo Enabled", AUTOEnabled);

        if(AUTOEnabled) { // if autoEnabled = false, these lines are not printed
            telemetry.addData("Auto target XR:", "%5.2f", currentXRTarget);
            telemetry.addData("Auto target XL:", "%5.2f", currentXLTarget);
            telemetry.addData("Auto target Y:", "%5.2f", currentYTarget);
           // telemetry.addData("heading OnTarget:", onHeading());
            telemetry.addData("lateral OnTarget:", atTarget());
        }
    }

    public void resetOdometry() { //this resets x or y encoder, also put seperately for modularity
        resetXREncoder();
        resetXLEncoder();
        resetYEncoder();
    }

}