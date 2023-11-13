//hardware class; defines each piece of hardware to be coded
//Imports: import all Motor classes and functions
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    //servos for airplane launch
    public CRServo rightLauncher;
    public CRServo leftLauncher;

    //Arm Motor
    public DcMotor armMotor;

    //extend time is 45 secs; multiply by 1000 for millisecond conversion
    public long extendTime = 1000 * 40;
    public boolean extendedState = false;

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


        //AIRPLANE LAUNCHER SERVOS
        rightLauncher = hardwareMap.get(CRServo.class, "rightLauncher");
        leftLauncher = hardwareMap.get(CRServo.class, "leftLauncher");

        rightLauncher.setDirection(CRServo.Direction.REVERSE);
        leftLauncher.setDirection(CRServo.Direction.FORWARD);

        rightLauncher.setPower(0);
        leftLauncher.setPower(0);


        //HOOK MOTOR
        leftHookMotor = hardwareMap.get(DcMotor.class, "hookMotor");
        rightHookMotor = hardwareMap.get(DcMotor.class, "hookMotor");
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
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        /*
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        */

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



    //spins servos to launch paper airplane



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
    public void hookMove(LinearOpMode teleop) {

        teleop.telemetry.addData("Hook Status: ", "Moving...");
        teleop.telemetry.update();

        if (!extendedState) {
            //if extended is false, extend & make true
            leftHookMotor.setPower(1);
            rightHookMotor.setPower(1);
            extendedState = true;
        } else if (extendedState) {
            //if extended is true, compress & make false
            leftHookMotor.setPower(-1);
            rightHookMotor.setPower(-1);
            extendedState = false;
        }

        teleop.sleep(extendTime);
        rightHookMotor.setPower(0);
        leftHookMotor.setPower(0);

        teleop.telemetry.addData("Hook Status: ", "Completed");
        teleop.telemetry.update();

        /*
        //checks if arm is already extended
        if (teleop.opModeIsActive()) {
            //might want to change the power later
            hookMotor.setTargetPosition(desiredPos);
            hookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hookMotor.setPower(1);

            if (hookMotor.getCurrentPosition() == hookMotor.getTargetPosition()) {
                hookMotor.setPower(0);
                hookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }
        */
    }

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
    }

    public void airplaneBandLauncher (LinearOpMode teleop) {

        teleop.telemetry.addData("Status: ", "Launching...");
        teleop.telemetry.update();
        //stopAll();    ?

        int moveTime = 2;





    }



    //Currently unused code
    /*
    public void airplaneSpinLaunch (LinearOpMode teleop) {
        //stops robot & displays status on driver hub
        teleop.telemetry.addData("Status: ", "Launching airplane");
        teleop.telemetry.update();
        stopAll();

        ElapsedTime spinTimer = new ElapsedTime();

        //how long & how fast servos spin
        int spinSeconds = 3;
        double launchPower = 0.5;

        //sets power to servos for how fast to be spinning when launch occurs

        rightLauncher.setPower(launchPower);
        leftLauncher.setPower(launchPower);

        //lets servos spin for however long needed
        while ((spinTimer.seconds() < spinSeconds) && teleop.opModeIsActive())  {
            teleop.telemetry.addData("Time passed: ", spinTimer.seconds());
            teleop.telemetry.update();

        }


        //stops launchers/servos
        rightLauncher.setPower(0);
        leftLauncher.setPower(0);

    }

    //Field centric drive
    public void fieldCentricDrive (double x, double y, double rx) {

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        //Calls non-centric drive but with field centric parameters
        robotCentricDrive(rotX, rotY, rx);

    }
    */

} // class RobotHardware