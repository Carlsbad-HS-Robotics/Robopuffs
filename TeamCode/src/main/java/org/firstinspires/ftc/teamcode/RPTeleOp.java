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

/*
Robopuffs 2023-2024: CenterStage
Author: Brielle McBarron
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="TeleOps")

public class RPTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        this.telemetry.addData("Status: ", "TeleOp Not Initialized");
        RobotHardware roboHardware = new RobotHardware(hardwareMap, this);
        roboHardware.initialize();
        this.telemetry.addData("Status: ", "TeleOp Initialized");
        this.telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
            roboHardware.armMovement(gamepad2.left_stick_y);
            roboHardware.clawGrab(gamepad2.left_trigger, gamepad2.right_trigger); //Claw: gamepad 2 triggers
            roboHardware.hookMove(gamepad1.y, gamepad1.a, gamepad1.start, gamepad2.start); //Hook: gamepad1 y up, a down (hold)
            roboHardware.launchAirplane(gamepad2.y); //2Y

            if (gamepad1.dpad_up) {
                roboHardware.hookSwing(true);
            }
            else if (gamepad1.dpad_down) {
                roboHardware.hookSwing(false);
            }

            //WRIST
            if (gamepad2.left_bumper) {
                roboHardware.wristServo.setPosition(1);
            }  //If it's in board position / towards back
            else if (gamepad2.right_bumper) {
                roboHardware.wristServo.setPosition(0.7);
            } //If it's in pickup position / towards front

            //IMU
            if (gamepad1.x) {
                roboHardware.reInitImu();
            }

        }
    }
}