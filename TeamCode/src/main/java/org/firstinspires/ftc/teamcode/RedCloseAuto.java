
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled

@Autonomous (name = "Red Close Side Autonomous", group = "Autonomous")
public class RedCloseAuto extends LinearOpMode {
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

        //1 mat = 915 milliseconds

        //NOTE: This autonomous is currently disabled

        //Grab the pixels
        /*
        roboHardware.clawGrab(1, 1);
        sleep(500);
        roboHardware.presetArm(false);
        sleep(500);
        roboHardware.goDrive(0.2, 1);
        roboHardware.turnRight();
        sleep(500);
        //Move backward
        roboHardware.goDrive(1.0, 1);
        roboHardware.stopDrive();
        sleep(500);
        //roboHardware.wristServo.setPosition(0);
        roboHardware.presetArm(true);
        sleep(500);
        roboHardware.clawGrab(0,0);
        sleep(500);
        roboHardware.presetArm(false);
        sleep(500);
        roboHardware.goDrive(0.5, -1);
        roboHardware.turnLeft();
        roboHardware.turnLeft();
        roboHardware.goDrive(1.5, -1);

         */

        roboHardware.goDrive(1.2, -1);

    }
}