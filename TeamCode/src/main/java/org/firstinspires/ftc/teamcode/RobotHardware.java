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

    /*
    public DcMotor hookMotor;
    public DcMotor flipMotor;
    public Servo airplaneLauncher; //servo for airplane launch
    public DcMotor armMotor;
    public DcMotor slideMotor;
    public Servo leftWristServo;
    public Servo rightWristServo;
    public Servo leftClaw;
    public Servo rightClaw;
     */ //Old motors & servos

    public IMU imu;
    public HardwareMap hardwareMap;

    public LinearOpMode teleOp;

    public double angleDiff = 0;

    public RobotHardware (HardwareMap hardwareMap, LinearOpMode teleOp) {
        this.hardwareMap = hardwareMap;
        this.teleOp = teleOp;
    }


    /*
    public void encoderInit () {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

     */ //Old motors & servos

    public void initializePowerMotor (DcMotor motor, boolean forward) {

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (forward) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (!forward) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    } //Initializes a DC Motor (mode, zero power behavior, direction)
    public void initializeEncoderMotor (DcMotor motor, boolean forward) {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (forward) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (!forward) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initialize() {

        //DRIVE MOTORS
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        initializePowerMotor(frontLeftMotor, true);
        initializePowerMotor(frontRightMotor, false);
        initializePowerMotor(backLeftMotor, true);
        initializePowerMotor(backRightMotor, false);

        /*
        //AIRPLANE LAUNCHER
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        airplaneLauncher.setDirection(Servo.Direction.FORWARD);

        //HOOK MOTOR
        hookMotor = hardwareMap.get(DcMotor.class, "hookMotor");
        initializeEncoderMotor(hookMotor,true);
        hookMotor.setTargetPosition(0);

        //HOOK FLIP MOTOR
        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        initializeEncoderMotor(flipMotor, true);

        //ARM MOTOR
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        initializeEncoderMotor(armMotor, true);

        //SLIDE MOTOR
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        initializeEncoderMotor(slideMotor, true);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //WRIST SERVOS
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        leftWristServo.setDirection(Servo.Direction.FORWARD);
        leftWristServo.scaleRange(0, 0.62);

        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        rightWristServo.setDirection(Servo.Direction.REVERSE);
        rightWristServo.scaleRange(0, 0.64);

        //CLAW SERVOS
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClaw.setDirection(Servo.Direction.FORWARD);

         */ //Old motors & servos

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

         */ //Test Field centric, if it doesn't work try uncommenting this

    } //initialize function
    public void reInitImu() {
        double newHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        angleDiff = -newHeading;
    } //Reinitialize IMU
    public void fieldCentricDrive (double x, double y, double rx) { //Removed ", LinearOpMode teleop" -- if it stopped working that might be why

        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        botHeading = botHeading+angleDiff;

        // Rotate the movement direction counter to the robot's rotation
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

        double speedModifier = 0.4;

        frontLeftPower = frontLeftPower - (frontLeftPower*speedModifier);
        frontRightPower = frontRightPower - (frontRightPower*speedModifier);
        backLeftPower = backLeftPower - (backLeftPower*speedModifier);
        backRightPower = backRightPower - (backRightPower*speedModifier);

        //Sets power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    //TELEOP FUNCTIONS

    /*

    int hookTargetPos = 0;

    static boolean hookCompressed = false;
    public void hookMove(boolean up,boolean down) {
        //13200 is max up

        if (up) {
            hookTargetPos = 1500;
        } else if (down) {
            hookTargetPos = 0;
        }

        hookMotor.setTargetPosition(hookTargetPos);
        hookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hookMotor.setPower(0.5);




    } //Move hoisting hooks
    int swingTargetPos = 0;
    public void hookSwing(boolean up, boolean down) {

        if (up) {
            swingTargetPos = 1500;
        } else if (down) {
            swingTargetPos = 0;
        } else {
            swingTargetPos = flipMotor.getCurrentPosition();
        }

        flipMotor.setTargetPosition(swingTargetPos);
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipMotor.setPower(1);
    }

    public void setWristPos(boolean dpad_up, boolean dpad_down, boolean dpad_left) {

        double wristPos = leftWristServo.getPosition();

        if (dpad_down) {
            wristPos = 0.3;
        } else if (dpad_left) {
            wristPos = 0.5;
        } else if (dpad_up) {
            wristPos = 1;
        }

        leftWristServo.setPosition(wristPos);
        rightWristServo.setPosition(wristPos);
    }
    int targetPos = 0;
    public void armMovement (double y) {

        targetPos = armMotor.getCurrentPosition();

        targetPos = targetPos + ((int) y * 500);

        armMotor.setTargetPosition(targetPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //TODO: Try removing this line and seeing how it runs
        armMotor.setPower(0.5);
    } //arm movement


    public void launchAirplane(boolean launchState, boolean start1, boolean start2) {

        //if y is pressed,
        if (launchState && !start1 && !start2) {
            stopDrive();
            airplaneLauncher.setPosition(0.3);
            airplaneLauncher.setPosition(0.8);
            teleOp.sleep(500);
            airplaneLauncher.setPosition(0.3);
        }

    } //launch the airplane

    public void clawGrab(float leftT, float rightT) {
        //Larger servo position = farther out

        if (leftT > 0.5) {
            leftClaw.setPosition(0.077);
        } else if (leftT <= 0.5) {
            leftClaw.setPosition(0.1);
        }

        if (rightT > 0.5) {
            rightClaw.setPosition(0.077);
        } else if (rightT <= 0.5) {
            rightClaw.setPosition(0.1);
        }

    } //Move the claws (individually)

    int sTargetPos = 0;

    public void slideMovement(double y) {
        sTargetPos = slideMotor.getCurrentPosition();

        sTargetPos = sTargetPos + ((int) y * 1000);

        slideMotor.setTargetPosition(sTargetPos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.1);
    }
    */ //Old motors & servos functions

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


    /*
    public void presetArm(boolean atZero) {
        //Direction true = towards front
        //Direction false = towards back
        if (atZero) {
            armMotor.setTargetPosition(0);
        }
        else if (!atZero) {
            armMotor.setTargetPosition(100);
        }
    } //Makes hook go forward or backward for a set time without stick control
     */

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