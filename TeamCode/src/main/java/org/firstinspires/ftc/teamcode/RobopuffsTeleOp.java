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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robopuffs TeleOp", group="TeleOps")

public class RobopuffsTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        this.telemetry.addData("Status: ", "TeleOp Not Initialized");
        RobotHardware roboHardware = new RobotHardware(hardwareMap);
        roboHardware.initialize(true);
        this.telemetry.addData("Status: ", "TeleOp Initialized");
        this.telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x, this);
            roboHardware.armMovement(gamepad2.left_stick_y);
            telemetry.addData("Encoder Position: ", roboHardware.armMotor.getCurrentPosition()); //constantly updates arm motor pos
            //CLAWS
            roboHardware.clawGrab(gamepad2.left_trigger, gamepad2.right_trigger);
            roboHardware.hookMove(this, gamepad1.y); //1Y
            roboHardware.launchAirplane(this, gamepad2.y); //2Y

            telemetry.addData("Left Claw Position:", roboHardware.leftClaw.getPosition());
            telemetry.addData("Right Claw Position:", roboHardware.rightClaw.getPosition());

            //WRIST
            if (gamepad2.left_bumper) {
                roboHardware.wristServo.setPosition(1);
            }  //If it's in pickup position
            else if (gamepad2.right_bumper) {
                roboHardware.wristServo.setPosition(0);
            } //If it's in board position

            //IMU
            if (gamepad1.x) {
                roboHardware.reinitImu();
            }

            //HOOK


            //AIRPLANE


            //HOOK SWING
            if (gamepad1.dpad_down) {
                roboHardware.flipMotor.setPower(0.23);
                sleep(600);
                roboHardware.flipMotor.setPower(0.001);
            } //DPAD DOWN = SWING DOWN
            else if (gamepad1.dpad_up) {
                roboHardware.flipMotor.setPower(-0.23);
                sleep(600);
                roboHardware.flipMotor.setPower(-0.001);
            } //DPAD UP = SWING UP

            if (gamepad2.dpad_left) {
                roboHardware.testEncoders();
            }

            telemetry.update();

            if (gamepad2.dpad_up) {
                roboHardware.presetArm(false, this);
            }
            else if (gamepad2.dpad_down) {
                roboHardware.presetArm(true, this);
            }

            telemetry.update();

        }
    }
}
