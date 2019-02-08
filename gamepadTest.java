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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
//barvaz
//Daniel :)

@TeleOp(name="gamepadTest", group="Linear Opmode")
//@Disabled
public class gamepadTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LeftDrive;
    private DcMotor RightDrive;

    @Override
    public void runOpMode() {

        elevetor elevetor_func = new elevetor();
        drive_class drive = new drive_class();
        slowMod slowDrive = new slowMod();
        relic_class relic = new relic_class();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LeftDrive = hardwareMap.dcMotor.get("Left_motor");
        RightDrive = hardwareMap.dcMotor.get("Right_motor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(gamepad1.right_trigger > 0){
                LeftDrive.setPower(gamepad1.right_trigger);
                RightDrive.setPower(-gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0) {
                LeftDrive.setPower(-gamepad1.left_trigger);
                RightDrive.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.left_stick_x > 0){
                LeftDrive.setPower(gamepad1.left_stick_x);
                RightDrive.setPower(gamepad1.left_stick_x);
            }
            else if(gamepad1.left_stick_x < 0){
                LeftDrive.setPower(-gamepad1.left_stick_x);
                RightDrive.setPower(-gamepad1.left_stick_x);
            }
            else {
                LeftDrive.setPower(0);
                RightDrive.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
