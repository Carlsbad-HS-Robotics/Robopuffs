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
import com.qualcomm.robotcore.hardware.DcMotor;
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

        //roboHardware.setWristPos(true, false, false); //puts wrist at position 0 (highest)
        roboHardware.hookMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            //GAMEPAD 1 CALLS
            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
            //roboHardware.hookMove(gamepad2.y, gamepad2.a);

            if (gamepad2.y) {
                roboHardware.hookMotor.setPower(1);
            }
            else if (gamepad2.a) {
                roboHardware.hookMotor.setPower(-1);
                //roboHardware.hookMotor.setTargetPosition(0);
            }
            else {
                roboHardware.hookMotor.setPower(0);
            }

            roboHardware.hookSwing(gamepad1.dpad_down, gamepad1.dpad_up);
            if (gamepad1.x) {
                roboHardware.reInitImu();
            } //Reset IMU

            //GAMEPAD 2 CALLS
            //roboHardware.armMovement(gamepad2.right_stick_y);

            if (gamepad2.right_stick_y > 0) {
                roboHardware.armMotor.setPower(0.5);
            }
            else if (gamepad2.right_stick_y < 0) {
                roboHardware.armMotor.setPower(-0.5);
            }
            else {
                roboHardware.armMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                roboHardware.flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad1.right_bumper) {
                roboHardware.flipMotor.setTargetPosition(-200);
                roboHardware.flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                roboHardware.flipMotor.setPower(1);
            }

            //roboHardware.hookMove(gamepad2.y,gamepad2.a);


            telemetry.addData("Arm pos: ", roboHardware.armMotor.getCurrentPosition());

            roboHardware.slideMovement(gamepad2.left_stick_y);

            roboHardware.clawGrab(gamepad2.left_trigger, gamepad2.right_trigger);
            roboHardware.setWristPos(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left);
            roboHardware.launchAirplane(gamepad2.b, gamepad1.start, gamepad2.start); //Airplane:

            /*
            Gamepad 1 Functions:
            * Movement (right stick)
            * Turning (right stick)
            * Hook swing (dpad up = vertical, dpad down = horizontal)
            * Reinitialize IMU (X)

            Gamepad 2 Functions:
            * Arm (right stick)
            * Wrist (Dpad up = up, Dpad left = horizontal, dpad down = down
            * Claws (left & right trigger for corresponding claws)
            * Hook compression/ extension (Y = extend, A = compress)
            * Airplane Launch (B)
            */

        }
    }
}