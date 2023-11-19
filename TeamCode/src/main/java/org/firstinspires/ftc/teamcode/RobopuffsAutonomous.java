
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "Test Autonomous", group = "Autonomous")
public class RobopuffsAutonomous extends LinearOpMode {

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

        roboHardware.goDrive(this);
        sleep(1000);
        telemetry.addData("Status: ", "Turning right...");
        roboHardware.turnRight(this);
        sleep(1000);
        telemetry.addData("Status: ", "Moving forward...");
        roboHardware.goDrive(this);

        telemetry.addData("Autonomous: ", "Finished");


    }




}
