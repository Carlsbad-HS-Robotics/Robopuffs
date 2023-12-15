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
        roboHardware.initialize();
        this.telemetry.addData("Status: ", "TeleOp Initialized");
        this.telemetry.update();

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        //gamepad1 is for robot movement/relocation
        //gamepad2 is for controlling the arm & airplane launcher

        /*
        roboHardware.wristServo.setPosition(0);
        roboHardware.leftClaw.setPosition(0.08);
        roboHardware.rightClaw.setPosition(0.09);

         */

        while (opModeIsActive()) {
            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x, this);
            roboHardware.armMovement(gamepad2.left_stick_y);

            if (gamepad2.left_bumper) {
                roboHardware.wristServo.setPosition(1);
            }  //If it's in pickup position
            else if (gamepad2.right_bumper) {
                roboHardware.wristServo.setPosition(0.36);
            } //If it's in board position

            if (gamepad2.dpad_up) {
                roboHardware.wristServo.setPosition(1);
            }


            if (gamepad2.a && !(gamepad2.start)) {
                roboHardware.clawGrab(this);
            } //claw clench

            /*if (gamepad2.x) {
                roboHardware.wristMovement(this);
            } //Wrist in-game movement

             */

            if (gamepad2.left_trigger > 0) {
                roboHardware.wristServo.setPosition(0);
                roboHardware.spitefulBooleans();
            } //set wrist to 0
            else if (gamepad2.right_trigger > 0) {
                roboHardware.wristServo.setPosition(1);
                roboHardware.spitefulBooleans();
            } //set wrist to 1

            if (gamepad1.y) {
                roboHardware.hookMove(this);
            } //Hook: 1Y
            if (gamepad2.y && !(gamepad2.start)) {
                telemetry.addData("Airplane: ", "Launching...");
                telemetry.update();
                roboHardware.launchAirplane(this);
                telemetry.addData("Airplane: ", "Launched");
                telemetry.update();
            } //Airplane: 2Y

            //Non-static/ preset Hook movement
            /*
            if (gamepad2.y) {
                roboHardware.rightHookMotor.setPower(1.0);

            } //UP
            else if (gamepad2.a) {
                roboHardware.rightHookMotor.setPower(-1.0);

            } //DOWN
            else {
                roboHardware.rightHookMotor.setPower(0);

            }

            //left
            if (gamepad2.dpad_up) {
                roboHardware.leftHookMotor.setPower(1.0);
            }
            else if (gamepad2.dpad_down) {
                roboHardware.leftHookMotor.setPower(-1.0);
            }
            else {
                roboHardware.leftHookMotor.setPower(0);
            }

            */ //Individual Hook Movement
            /*
            if (gamepad2.y) {
                roboHardware.rightHookMotor.setPower(1.0);
                roboHardware.leftHookMotor.setPower(1.0);

            } //UP
            else if (gamepad2.a) {
                roboHardware.rightHookMotor.setPower(-1.0);
                roboHardware.leftHookMotor.setPower(-1.0);

            } //DOWN
            else {
                roboHardware.rightHookMotor.setPower(0);
                roboHardware.leftHookMotor.setPower(0);
            }

             */ //Both Hoist Hooks
        }
    }
}
