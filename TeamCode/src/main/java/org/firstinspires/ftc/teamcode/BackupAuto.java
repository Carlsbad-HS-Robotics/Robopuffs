
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "Close Basic Back Up", group = "Autonomous")
public class BackupAuto extends LinearOpMode {

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
        roboHardware.goDrive(this,2.3, -1);

    }
}