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

        //int targetPos = 0;

        roboHardware.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roboHardware.armMotor.setDirection(DcMotor.Direction.REVERSE);
        //TODO: Try putting lines 57 & 58 (above two lines) into initialize instead

        while (opModeIsActive()) {
            //Gamepad 1 Calls
            roboHardware.fieldCentricDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
            roboHardware.hookMove(gamepad1.y, gamepad1.a, gamepad1.start, gamepad2.start);
            if (gamepad1.x) {
                roboHardware.reInitImu();
            } //Reset IMU

            //Gamepad 2 Calls
            roboHardware.armMovement(gamepad2.right_stick_y);
            roboHardware.clawGrab(gamepad2.left_trigger, gamepad2.right_trigger);
            roboHardware.launchAirplane(gamepad2.y);
            //roboHardware.hookSwing(gamepad2.dpad_up, gamepad2.dpad_down); //TODO: Why is this causing problem
            if (gamepad2.x) {
                roboHardware.encoderInit();
            }

            telemetry.addData("Arm Encoder Position: ", roboHardware.armMotor.getCurrentPosition());

            //TODO: find how many encoder wires do we have?
            //Ideally want to use encoders for: arm (done), slider, hook, and hook swing.
            //Priority: 1. Slider, 2. Hook 3. Swing

            //TODO: take off tape from arm motor



            //TODO: Test servos and see direction, position, etc.
            if (gamepad2.right_bumper) {
                roboHardware.leftWristServo.setPosition(0.5); //Less = [Direction]
            } else if (gamepad2.left_bumper) {
                roboHardware.leftWristServo.setPosition(0.7); //More = [Direction]
            }

            telemetry.update();


            /*
            Gamepad 1 Functions:
            * Movement (right stick)
            * Turning (right stick)
            * Hook compression/ extension (y = extend, a = compress)
            * Reinitialize (x)

            Gamepad 2 Functions:
            * Arm (right stick)
            * Hook swing (dpad up = vertical, dpad down = horizontal)
            * Claws (left & right trigger for corresponding claws)
            * Airplane Launch (y)
            */

        }
    }
}