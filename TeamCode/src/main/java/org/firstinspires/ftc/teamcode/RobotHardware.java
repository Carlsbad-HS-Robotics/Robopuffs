//hardware class; defines each piece of hardware to be coded
//Imports: import all Motor classes and functions
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //defines hardware pieces
    //public ElapsedTime runtime = new ElapsedTime();
    //drive motors:
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    //hook motors
    public DcMotor rightHookMotor;
    public DcMotor leftHookMotor;

    //servo for airplane launch
    public Servo airplaneLauncher;

    //Arm Motor
    public DcMotor armMotor;
    public static boolean extendedState = false;

    public long moveTime = 5000;

    //private BNO055IMU imu;
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

        //AIRPLANE LAUNCHER
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        airplaneLauncher.setDirection(Servo.Direction.FORWARD);


        //HOOK MOTOR
        leftHookMotor = hardwareMap.get(DcMotor.class, "leftHookMotor");
        rightHookMotor = hardwareMap.get(DcMotor.class, "rightHookMotor");
        leftHookMotor.setPower(0);
        rightHookMotor.setPower(0);
        leftHookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHookMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightHookMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //ARM MOTOR

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        */ //imu code

    } //init function

    public void robotCentricDrive (double x, double y, double rx) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x - rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftPower = frontLeftPower - (frontLeftPower*0.3);
        frontRightPower = frontRightPower - (frontRightPower*0.3);
        backLeftPower = backLeftPower - (backLeftPower*0.3);
        backRightPower = backRightPower - (backRightPower*0.3);

        //Sets power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    } //Robot centric drive

    public void stopAll() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    } //Stops all motors/servos

    public void hookMove(LinearOpMode teleop) {

        teleop.telemetry.addData("Hook: ", "Moving...");
        teleop.telemetry.update();

        if (!extendedState) {
            //if extended is false, extend & make true
            leftHookMotor.setPower(1);
            rightHookMotor.setPower(1);
        } else if (extendedState) {
            //if extended is true, compress & make false
            leftHookMotor.setPower(-1);
            rightHookMotor.setPower(-1);
        }

        extendedState = !extendedState;
        teleop.sleep(moveTime);
        rightHookMotor.setPower(0);
        leftHookMotor.setPower(0);

        teleop.telemetry.addData("Hook: ", "Moved");
        teleop.telemetry.update();

    } //Move hoisting hooks

    public void armMovement (double y) {

        double fullPower = 0.9;
        double halfPower = 0.4;

        if (y <= .6 && y>0) {
            //half power forward
            armMotor.setPower(halfPower);
        } else if (y > .6) {
            //full power forward
            armMotor.setPower(fullPower);
        } else if (y >= -0.6 && y<0) {
            //half power backward
            armMotor.setPower(-halfPower);
        } else if (y < -0.6) {
            //full power forward
            armMotor.setPower(-fullPower);
        } else {
            armMotor.setPower(0);
        }
    } //arm movement

    //AUTONOMOUS FUNCTIONS

    double driveSpeed = 0.5;

    public void goDrive(LinearOpMode teleop, long driveTime) {
        backLeftMotor.setPower(driveSpeed);
        backRightMotor.setPower(driveSpeed);
        frontLeftMotor.setPower(driveSpeed);
        frontRightMotor.setPower(driveSpeed);
        teleop.sleep(driveTime);
        stopDrive();

    } //auto drive for x time
    public void stopDrive() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    } //stop
    public void turnRight(LinearOpMode teleop) {
        backLeftMotor.setPower(driveSpeed);
        backRightMotor.setPower(-driveSpeed);
        frontLeftMotor.setPower(driveSpeed);
        frontRightMotor.setPower(-driveSpeed);
        teleop.sleep(830);
        stopDrive();
    }
    public void turnLeft(LinearOpMode teleop) {
        backLeftMotor.setPower(-driveSpeed);
        backRightMotor.setPower(driveSpeed);
        frontLeftMotor.setPower(-driveSpeed);
        frontRightMotor.setPower(driveSpeed);
        teleop.sleep(830);
        stopDrive();
    }

    /*
    //Field centric drive
    public void fieldCentricDrive (double x, double y, double rx) {

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        //Calls non-centric drive but with field centric parameters
        robotCentricDrive(rotX, rotY, rx);

    }
    */ //field centric drive

} // class RobotHardware