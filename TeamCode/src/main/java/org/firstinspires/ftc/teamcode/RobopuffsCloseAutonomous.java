
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
        roboHardware.initialize();
        this.telemetry.addData("Status: ", "Initialized");
        this.telemetry.update();

        waitForStart();
        autoTime.reset();


        roboHardware.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        roboHardware.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        roboHardware.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        roboHardware.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //6 mats

        //1 mat = 915 milliseconds
        sleep(500);
        roboHardware.backLeftMotor.setPower(0.5);
        roboHardware.backRightMotor.setPower(0.5);
        roboHardware.frontLeftMotor.setPower(0.5);
        roboHardware.frontRightMotor.setPower(0.5);
        sleep(915*2);
        roboHardware.stopDrive();


    }
}