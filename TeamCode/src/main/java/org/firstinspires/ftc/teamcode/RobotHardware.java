//hardware class; defines each piece of hardware to be coded
//Imports: import all Motor classes and functions
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //public ElapsedTime runtime = new ElapsedTime();

    //drive motors:
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    //hook motor
    public DcMotor hookMotor;
    public DcMotor flipMotor;

    public int armMotorCPR;

    //servo for airplane launch
    public Servo airplaneLauncher;

    //Arm Motor
    public DcMotor armMotor;
    public static boolean extendedState = false;

    public long moveTime = 5100; //Hook movement time (in milliseconds)

    public Servo wristServo;
    public Servo leftClaw;
    public Servo rightClaw;

    public IMU imu;
    public HardwareMap hardwareMap;

    public double angleDiff = 0;

    public RobotHardware (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initialize(boolean imuInit) {

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
        hookMotor = hardwareMap.get(DcMotor.class, "hookMotor");
        hookMotor.setPower(0);
        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hookMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //HOOK FLIP MOTOR
        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        flipMotor.setPower(0);
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //ARM MOTOR
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //WRIST SERVO
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        wristServo.setDirection(Servo.Direction.FORWARD);
        wristServo.scaleRange(0, 1);

        //CLAW SERVOS
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        leftClaw.setDirection(Servo.Direction.FORWARD);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        if (imuInit) {
            initImu();
        } else {
            imu = hardwareMap.get(IMU.class, "imu");
        }

    } //init function

    public void initImu() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }  //Initialize IMU
    public void reinitImu() {
        double newHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        angleDiff = -newHeading;

    } //Reinitialize IMU
    public void fieldCentricDrive (double x, double y, double rx, LinearOpMode teleop) {
        //teleop.telemetry.addData("gamepadx: ", x);
        //teleop.telemetry.addData("gamepady: ", y);

        // Read inverse IMU heading, as the IMU heading is CW positive
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = botHeading+angleDiff;
        //teleop.telemetry.addData("botHeading: ", botHeading);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
        double rotY = -x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        //teleop.telemetry.addData("rotx: ", rotX);
        //teleop.telemetry.addData("roty: ", rotY);
        //rotX = -rotX;
        //teleop.telemetry.update();
        rotX = rotX * 1.1;  // Counteract imperfect strafing

        //rotY = -rotY;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        robotCentricDrive(rotX,rotY,rx);

    }
    public void robotCentricDrive (double x, double y, double rx) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x - rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftPower = frontLeftPower - (frontLeftPower*0.4);
        frontRightPower = frontRightPower - (frontRightPower*0.4);
        backLeftPower = backLeftPower - (backLeftPower*0.4);
        backRightPower = backRightPower - (backRightPower*0.4);

        //Sets power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    //TELEOP FUNCTIONS

    public void hookMove(LinearOpMode teleop, boolean y, boolean a, boolean start1, boolean start2) {
        //Extended State true = not extended

        if (y) {
            hookMotor.setPower(1);
        } else if (a && !start1 && !start2) {
            hookMotor.setPower(-1);
        } else {
            hookMotor.setPower(0);
        }

        /*

        if (y) {
            stopDrive();
            //clawGrab(1,1);
            if (extendedState) {
                //NOT extended
                hookMotor.setPower(-1);
                teleop.sleep(moveTime);
            }
            else if (!extendedState) {

                hookMotor.setPower(1);
                teleop.sleep(moveTime);
            } //If it's extended fully or halfway

            hookMotor.setPower(0);
            extendedState = !extendedState;
        }

         */


    } //Move hoisting hooks
    public void hookSwing(LinearOpMode teleop, boolean up, boolean down) {
        if (up) {
            flipMotor.setPower(-0.30);
            teleop.sleep(800);
            flipMotor.setPower(0);
        }
        if (down) {
            flipMotor.setPower(0.30);
            teleop.sleep(800);
            flipMotor.setPower(0);
        }
    }

    public void armMovement (double y) {

        double fullPower = 0.9;
        double mediumPower = 0.4;
        double minimalPower = 0.2;

        if (y > 0.7) {
            armMotor.setPower(fullPower);
        } //fast forward
        else if (y < -0.7) {
            armMotor.setPower(-fullPower);
        } //fast backward
        else if (y > 0.25) {
            armMotor.setPower(mediumPower);
        } //medium forward
        else if (y < -0.25) {
            armMotor.setPower(-mediumPower);
        } //medium backward
        else if (y > 0) {
            armMotor.setPower(minimalPower);
        } //slowly forward
        else if (y < 0) {
            armMotor.setPower(-minimalPower);
        } //slowly backward
        else {
            armMotor.setPower(0);
        } //no movement
    } //arm movement

    public void launchAirplane(LinearOpMode teleop, boolean launchState) {
        if (launchState) {
            airplaneLauncher.setPosition(0.3);
            //sleep(1000);
            airplaneLauncher.setPosition(0.8);
            teleop.sleep(1000);
            airplaneLauncher.setPosition(0.3);
        }
    } //launch the airplane

    public void clawGrab(float leftT, float rightT) {
        //Larger servo position = farther out

        if (leftT > 0.5) {
            leftClaw.setPosition(0.047);
        } else if (leftT <= 0.5) {
            leftClaw.setPosition(0.09);
        }

        if (rightT > 0.5) {
            rightClaw.setPosition(0.059);
        } else if (rightT <= 0.5) {
            rightClaw.setPosition(0.09);
        }

    } //Move the claws (individually)

    public void presetArm(boolean direction, LinearOpMode teleop) {
        //Direction true = towards front
        //Direction false = towards back
        if (direction) {
            armMotor.setPower(1);
            teleop.sleep(1000); //Should be 1500
            armMotor.setPower(0);
        }
        else if (!direction) {
            armMotor.setPower(-1);
            teleop.sleep(1000); //should be 1500
            armMotor.setPower(0);
        }
    }

    public void testEncoders(boolean dir) {
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (dir) {
            armMotor.setTargetPosition(-300);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (!dir) {
            armMotor.setTargetPosition(300);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


    }
    public void resetEncoderZero () {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //AUTONOMOUS FUNCTIONS
    double driveSpeed = -0.5;
    public void goDrive(LinearOpMode teleop, double numMats, int dir) {
        //915 ms is one mat
        backLeftMotor.setPower(driveSpeed*dir);
        backRightMotor.setPower(driveSpeed*dir);
        frontLeftMotor.setPower(driveSpeed*dir);
        frontRightMotor.setPower(driveSpeed*dir);
        int driveTime = (int) numMats * 915;
        teleop.sleep(driveTime);
        stopDrive();

    } //auto drive for x time
    public void stopDrive() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    } //stop

    int turnTime = 820;
    public void turnRight(LinearOpMode teleop) {
        backLeftMotor.setPower(-driveSpeed);
        backRightMotor.setPower(driveSpeed);
        frontLeftMotor.setPower(-driveSpeed);
        frontRightMotor.setPower(driveSpeed);
        teleop.sleep(turnTime);
        stopDrive();
    }
    public void turnLeft(LinearOpMode teleop) {
        backLeftMotor.setPower(driveSpeed);
        backRightMotor.setPower(-driveSpeed);
        frontLeftMotor.setPower(driveSpeed);
        frontRightMotor.setPower(-driveSpeed);
        teleop.sleep(turnTime);
        stopDrive();
    }

} // class RobotHardware