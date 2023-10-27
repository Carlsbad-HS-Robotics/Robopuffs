//hardware class; defines each piece of hardware to be coded
//Imports: import all Motor classes and functions
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotHardware {

    //defines hardware pieces
    public ElapsedTime runtime = new ElapsedTime();
    //drive motors:
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    //hoist motor
    public DcMotor hoistMotor;

    //servos for airplane launch
    public CRServo rightLauncher;
    public CRServo leftLauncher;

    //what does this do?
    private BNO055IMU imu;
    public HardwareMap hardwareMap;

    public RobotHardware (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initialize() {

        //DRIVE MOTORS
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //set starting motor power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //...RunMode.RUN_TO_POSITION - for autonomous when you want to use encoder positions
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ZeroPowerBehavior: When the power is 0, the robot will not move
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //...zeroPowerBehavior.float we want it to stop exactly when set to zero
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hoistMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //AIRPLANE LAUNCHER SERVOS
        CRServo rightLauncher = hardwareMap.get(CRServo.class, "rightLauncher");
        CRServo leftLauncher = hardwareMap.get(CRServo.class, "leftLauncher");

        rightLauncher.setDirection(CRServo.Direction.FORWARD);
        leftLauncher.setDirection(CRServo.Direction.REVERSE);

        rightLauncher.setPower(0);
        leftLauncher.setPower(0);


        //HOIST MOTOR
        hoistMotor = hardwareMap.get(DcMotor.class, "hoistMotor");
        hoistMotor.setPower(0);
        hoistMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoistMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

    } //init function

    //Robot centric drive; currently the drive we're using
    public void robotCentricDrive (double x, double y, double rx) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x - rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        //Sets power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    //Field centric drive; Not currently in use
    public void fieldCentricDrive (double x, double y, double rx) {

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        //Calls non-centric drive but with field centric parameters
        robotCentricDrive(rotX, rotY, rx);

    }

    //spins servos to launch paper airplane
    public void airplaneLauncher (LinearOpMode teleop) {

        //stops robot & displays status on driver hub
        stopAll();
        teleop.telemetry.addData("Status: ", "Launching airplane");
        teleop.telemetry.update();

        ElapsedTime spinTimer = new ElapsedTime();

        //how long & how fast servos spin
        int spinSeconds = 2 * 1000;
        double launchPower = 0.5;

        //sets power to servos for how fast to be spinning when launch occurs
        rightLauncher.setPower(launchPower);
        leftLauncher.setPower(launchPower);

        //lets servos spin for however long needed
        while ((spinTimer.milliseconds() < spinSeconds) && teleop.opModeIsActive())  {
            ;
        }

        //stops launchers/servos
        rightLauncher.setPower(0);
        leftLauncher.setPower(0);

    }

    //Stops the robot whatever it's doing
    public void stopAll() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        rightLauncher.setPower(0);
        leftLauncher.setPower(0);
    }

    //hoists robot up onto truss
    public void hoistBot(LinearOpMode teleop) {

        //write code to make the motor run as long as needed to extend as little as possibly needed?
        //if that makes sense
    }

} // class RobotHardware