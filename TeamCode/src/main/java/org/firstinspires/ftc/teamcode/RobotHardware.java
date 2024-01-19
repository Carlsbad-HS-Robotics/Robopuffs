/*
Robopuffs 2023-2024: CenterStage
Author: Brielle McBarron
 */
//hardware class; defines each piece of hardware to be coded
//Imports: import all Motor classes and functions
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //Drive Motors:
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor hookMotor; //hook motor
    public DcMotor flipMotor;
    public Servo airplaneLauncher; //servo for airplane launch

    public DcMotor armMotor; //Arm Motor

    //public long moveTime = 5100; //Hook movement time (in milliseconds)
    //I MIGHT NEED THIS KAREN I PROMISE

    public Servo wristServo;
    public Servo leftClaw;
    public Servo rightClaw;
    public IMU imu;
    public HardwareMap hardwareMap;

    public LinearOpMode teleOp;

    public double angleDiff = 0;

    public RobotHardware (HardwareMap hardwareMap, LinearOpMode teleOp) {
        this.hardwareMap = hardwareMap;
        this.teleOp = teleOp;
    }

    public void initializeMotor (DcMotor motor, boolean forward) {

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (forward) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (!forward) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    } //Initializes a DC Motor
    public void initialize() {

        //DRIVE MOTORS
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        initializeMotor(frontLeftMotor, true);
        initializeMotor(frontRightMotor, false);
        initializeMotor(backLeftMotor, true);
        initializeMotor(backRightMotor, false);

        //AIRPLANE LAUNCHER
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        airplaneLauncher.setDirection(Servo.Direction.FORWARD);

        //HOOK MOTOR
        hookMotor = hardwareMap.get(DcMotor.class, "hookMotor");
        initializeMotor(hookMotor,true);

        //HOOK FLIP MOTOR
        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        initializeMotor(flipMotor, true);

        //ARM MOTOR
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        initializeMotor(armMotor, true);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        /*
        if (imuInit) {
            initImu();
        } else {
            imu = hardwareMap.get(IMU.class, "imu");
        }
        //Test Field centric, if it doesn't work try putting 140 into code
         */

    } //init function
    public void reInitImu() {
        double newHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        angleDiff = -newHeading;
    } //Reinitialize IMU
    public void fieldCentricDrive (double x, double y, double rx) { //Removed ", LinearOpMode teleop" -- if it stopped working that might be why

        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = botHeading+angleDiff;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
        double rotY = -x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

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

    public void hookMove(boolean y, boolean a, boolean start1, boolean start2) {

        if (y) {
            hookMotor.setPower(1);
        } else if (a && !start1 && !start2) {
            hookMotor.setPower(-1);
        } else {
            hookMotor.setPower(0);
        }

        //Extended State true = not extended
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

    // TODO consider changing the approach to not 'sleep' -- talk to mentors
    public void hookSwing(boolean up) {
        if (up) {
            flipMotor.setPower(-0.30);
        }
        else if (!up) { //down
            flipMotor.setPower(0.30);
        }
        teleOp.sleep(800);
        flipMotor.setPower(0);
    }

    // TODO reconsider input->output function, talk to mentors
    public void armMovement (double y) {

        double fullPower = 0.9;
        double mediumPower = 0.4;
        double minimalPower = 0.2;

        double multiplier = y / Math.abs(y); //Determines if positive or negative power
        //Note: Above should be positive 1 if positive power (0.8 / 0.8 = 1) & negative 1 if negative power (-0.8 / 0.8 = -1)
        y = Math.abs(y);

        if (y > 0.7) {
            armMotor.setPower(fullPower * multiplier);
        } //fast
        else if (y > 0.25) {
            armMotor.setPower(mediumPower * multiplier);
        } //medium
        else if (y > 0) {
            armMotor.setPower(minimalPower);
        } // backward
        else {
            armMotor.setPower(0);
        } //no movement
    } //arm movement

    public void launchAirplane(boolean launchState) {
        stopDrive();
        airplaneLauncher.setPosition(0.3);
        airplaneLauncher.setPosition(0.8);
        teleOp.sleep(500);
        airplaneLauncher.setPosition(0.3);
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

    //AUTONOMOUS FUNCTIONS

    final double AUTODRIVESPEED = -0.5; //The speed of Oswald's drive during Autonomous
    public void goDrive(double numMats, int dir) {
        //915 ms is one mat
        backLeftMotor.setPower(AUTODRIVESPEED*dir);
        backRightMotor.setPower(AUTODRIVESPEED*dir);
        frontLeftMotor.setPower(AUTODRIVESPEED*dir);
        frontRightMotor.setPower(AUTODRIVESPEED*dir);
        int driveTime = (int) numMats * 915;
        teleOp.sleep(driveTime);
        stopDrive();

    } //auto drive for __ mats forward/backward
    public void stopDrive() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    } //stop robot movement
    public void presetArm(boolean direction) {
        //Direction true = towards front
        //Direction false = towards back
        if (direction) {
            armMotor.setPower(1);
            teleOp.sleep(1000); //Should be 1500
            armMotor.setPower(0);
        }
        else if (!direction) {
            armMotor.setPower(-1);
            teleOp.sleep(1000); //should be 1500
            armMotor.setPower(0);
        }
    } //Makes hook go forward or backward for a set time without stick control
    int turnTime = 820; //How long robot turns for
    public void turnRight() {
        backLeftMotor.setPower(-AUTODRIVESPEED);
        backRightMotor.setPower(AUTODRIVESPEED);
        frontLeftMotor.setPower(-AUTODRIVESPEED);
        frontRightMotor.setPower(AUTODRIVESPEED);
        teleOp.sleep(turnTime);
        stopDrive();
    }
    public void turnLeft() {
        backLeftMotor.setPower(AUTODRIVESPEED);
        backRightMotor.setPower(-AUTODRIVESPEED);
        frontLeftMotor.setPower(AUTODRIVESPEED);
        frontRightMotor.setPower(-AUTODRIVESPEED);
        teleOp.sleep(turnTime);
        stopDrive();
    }

} // class RobotHardware