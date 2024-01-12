
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "Close Side Autonomous", group = "Autonomous")
public class RobopuffsCloseAutonomous extends LinearOpMode {
    private ElapsedTime autoTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        this.telemetry.addData("Status: ", "Not Initialized");
        this.telemetry.update();
        RobotHardware roboHardware = new RobotHardware(hardwareMap);
        roboHardware.initialize(true);
        this.telemetry.addData("Status: ", "Initialized");
        this.telemetry.update();

        waitForStart();
        autoTime.reset();

        //1 mat = 915 milliseconds


        //Grab the pixels
        roboHardware.clawGrab(1, 1);
        roboHardware.presetArm(true,this);
        roboHardware.turnLeft(this);
        sleep(500);
        //Move backward
        roboHardware.backLeftMotor.setPower(-0.5);
        roboHardware.backRightMotor.setPower(-0.5);
        roboHardware.frontLeftMotor.setPower(-0.5);
        roboHardware.frontRightMotor.setPower(-0.5);
        sleep(915*2-(915/4));
        roboHardware.stopDrive();


    }
}