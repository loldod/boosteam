/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RevServoTest", group="Linear Opmode")
//@Disabled
public class TestServoRev extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Servo RightServo;
    private Servo LeftServo;

    double pos1 = 0.5;
    double pos2 = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RightServo = hardwareMap.servo.get("RS");
        LeftServo = hardwareMap.servo.get("LS");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {

            RightServo.setPosition(pos1);
            LeftServo.setPosition(pos2);

            if (gamepad1.right_trigger > 0){
                pos1 = 0.3;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.right_bumper){
                pos1 = 0.1;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.left_trigger > 0){
                pos1 = 0.7;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.left_bumper){
                pos1 = 0.9;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.x){
                pos1 = 0.5;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.dpad_down){
                pos1 = 0.2;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.dpad_up){
                pos1 = 0.8;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.dpad_right){
                pos1 = 0.4;
                pos2 = 1 - pos1;
            }

            else if (gamepad1.dpad_left){
                pos1 = 0.6;
                pos2 = 1 - pos1;
            }

            telemetry.addData("pos1: ", pos1);
            telemetry.addData("pos2: ", pos2);
            telemetry.update();
        }
    }
}

