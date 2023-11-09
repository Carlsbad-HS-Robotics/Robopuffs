/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test TeleOp", group="Test Group")

//@Disabled //Comment out @Disabled to make opmode available
public class RobopuffsTestTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        this.telemetry.addData("Status: ", "Not Initialized");
        RobotHardware roboHardware = new RobotHardware(hardwareMap);
        roboHardware.initialize();
        this.telemetry.addData("Status: ", "Initialized");
        this.telemetry.update();

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        //gamepad1 is for robot movement/relocation
        //gamepad2 is for controlling the arm & airplane launcher
        while (opModeIsActive()) {
            roboHardware.robotCentricDrive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

            //airplane launch
            if (gamepad2.a) {
                this.telemetry.addData("Status: ", "Airplane Launching...");
                this.telemetry.update();
                roboHardware.airplaneLauncher(this);
                this.telemetry.addData("Status: ", "Airplane Launched.");
                this.telemetry.update();
            }


            //extend the suspension hook
            /*
            if (gamepad2.y) {
                roboHardware.hookMove(this, 200);
            }

             */



            if (gamepad2.y) {
                roboHardware.hookMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                roboHardware.hookMotor.setPower(5);
            } else if (gamepad2.x) {
                roboHardware.hookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                roboHardware.hookMotor.setPower(5);
            } else {
                roboHardware.hookMotor.setPower(0);
            }

            /*
            //lift the robot
            if (gamepad2.x) {
                roboHardware.hookMove(this, 1000);
            }
            //store the suspension hook
            if (gamepad2.b) {
                roboHardware.hookMove(this, 0);
            }

             */

        }
    }
}
