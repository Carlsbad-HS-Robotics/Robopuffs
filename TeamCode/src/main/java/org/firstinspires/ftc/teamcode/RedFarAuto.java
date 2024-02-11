
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "Red Far Side Autonomous", group = "Autonomous")
public class RedFarAuto extends LinearOpMode {

    private ElapsedTime autoTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        this.telemetry.addData("Status: ", "Not Initialized");
        this.telemetry.update();
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        this.telemetry.addData("Status: ", "Initialized");
        this.telemetry.update();

        waitForStart();
        autoTime.reset();


        roboHardware.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        roboHardware.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        roboHardware.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        roboHardware.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //1 mat = 915 milliseconds
        //roboHardware.clawGrab(1,1);
        sleep(500);
        //roboHardware.presetArm(false, this);
        sleep(500);
        roboHardware.goDrive(2.2, 1); //first drive forward
        sleep(500);
        roboHardware.turnRight(); //Turn to truss; turns left not red for some reason
        sleep(500);
        roboHardware.hookSwing(false, true);
        roboHardware.armMovement(0.5);
        sleep(500);
        roboHardware.goDrive( 4, 1); //Move to backstage

    }




}
