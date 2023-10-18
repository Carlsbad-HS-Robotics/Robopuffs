package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//do i need to extend anything
public class AirplaneLauncher {

    //copied: THIS WILL BE CHANGED I'm just copying it as like a frame for the Airplane Launcher code
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        RobotHardware roboHardware = new RobotHardware(hardwareMap);
        roboHardware.initialize();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            roboHardware.robotCentricDrive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        }
    }

}
