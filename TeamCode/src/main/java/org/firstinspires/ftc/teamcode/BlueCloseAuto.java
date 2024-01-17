
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
        RobotHardware roboHardware = new RobotHardware(hardwareMap);
        roboHardware.initialize(true);
        this.telemetry.addData("Status: ", "Initialized");
        this.telemetry.update();

        waitForStart();
        autoTime.reset();

        //1 mat = 915 milliseconds












        //Grab the pixels
        roboHardware.presetArm(true,this); //Put arm down
        sleep(500);
        roboHardware.clawGrab(1, 1); //close claw (grab pixel)
        sleep(500);
        roboHardware.presetArm(false,this); //lift arm
        sleep(500);
        roboHardware.goDrive(this, 0.2, 1); //move slightly forward
        roboHardware.turnRight(this); //turn towards backstage
        roboHardware.goDrive(this, 1, 1); //move to backstage
        roboHardware.wristServo.setPosition(0);
        roboHardware.presetArm(true,this); //put down arm
        sleep(500);
        roboHardware.clawGrab(0,0); //open claw (let go of pixel)
        sleep(500);
        roboHardware.presetArm(false,this); //lift arm
        sleep(500);
        roboHardware.turnLeft(this); //turn 180
        roboHardware.turnLeft(this); //turn 180
        sleep(500);
        roboHardware.goDrive(this, 1.0, -1); //go backwards into backstage
        roboHardware.stopDrive();
        sleep(500);

    }
}