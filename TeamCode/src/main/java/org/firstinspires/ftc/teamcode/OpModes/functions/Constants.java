package org.firstinspires.ftc.teamcode.OpModes.functions;

public class Constants {

    public static final boolean DASHBOARD_ENABLED = true;// this needs to be false for competitions!!
    public static final boolean TELEMETRY_ENABLED = true;

    public static final double TICKS_TO_INCHES = Math.PI * 48 / (25.4 * 2000); // gotta figure out what means what for this also!!!!!
    public static final double SPEED_RATIO = 0.7;  // Use this to slow down robot
    public static final double FORWARD_DRIVE_DISTANCE = 20;  // distance in inches
    public static final double DRIVETRAIN_TICKS = 336.00; // measured in ticks
    /* encoder resolution (ticks) = exact gearbox ratio (#:1) * bare motor ticks (ticks)
    ticks for our drivetrain is 336.00 (12:1 gearbox * 28 ticks for bare motor)
    ticks for our intake roller is 146.44 (5:1 gearbox * 28 ticks for bare motor)
    double the ticks to double the rotations, or double the rotations to do the same */
    public static final double HEADING_ERROR = 1.0; //FIND OUR FOR OUR OWN BOT
    public static final double DRIVE_PID_ERROR = 0.1; //FIND NUMBER FOR OUR BOT
    public static final double WHEEL_DIAMETER = 3.7795; // measured in inches (95cm)
    /* public static final double one rotation of the wheeel  = Math.PI * [wheel diameter]
    so theroretically [insert result here] ticks = pi/90 inches, so ticks-to-inches = ([encoder value] x pi/[wheel diamater/4])]
    wheel diameter is 96mm or 3.7795 inches x pi = 11.8736494 / 4 = 2.96841235 */

    // directions
    public static final double forward = 180; // north
    public  static final double back = 0; // south
    public  static final double right = 90; // east
    public static  final double left = -90; //west

    public static double POWER;

    //CONSTANTS FOR CUSTOMDESIGNAUTOIDEA
    public static double moveforward = 10;
    public static double movebackward = 10;
    public static double strafe = 10;
    public static double rotate = 10;
}

