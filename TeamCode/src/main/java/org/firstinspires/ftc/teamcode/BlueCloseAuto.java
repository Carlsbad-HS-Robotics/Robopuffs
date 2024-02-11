
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled

@Autonomous (name = "Blue Close Side Autonomous", group = "Autonomous")
public class BlueCloseAuto extends LinearOpMode {
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
        roboHardware.presetArm(true); //Put arm down
        sleep(500);
        roboHardware.clawGrab(1, 1); //close claw (grab pixel)
        sleep(500);
        roboHardware.presetArm(false); //lift arm
        sleep(500);
        roboHardware.goDrive(0.2, 1); //move slightly forward
        roboHardware.turnRight(); //turn towards backstage
        roboHardware.goDrive( 1, 1); //move to backstage
        //roboHardware.wristServo.setPosition(0);
        roboHardware.presetArm(true); //put down arm
        sleep(500);
        roboHardware.clawGrab(0,0); //open claw (let go of pixel)
        sleep(500);
        roboHardware.presetArm(false); //lift arm
        sleep(500);
        roboHardware.turnLeft(); //turn 180
        roboHardware.turnLeft(); //turn 180
        sleep(500);
        roboHardware.goDrive(1.0, -1); //go backwards into backstage
        roboHardware.stopDrive();
        sleep(500);

         */

        roboHardware.goDrive(1.2, -1);

    }
}