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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="HaraForKid", group="Linear Opmode")
//@Disabled
public class HaraKids extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LeftMotor;
    private DcMotor RightMotor;

    private Servo servo;

    DistanceSensor sensor_distance;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LeftMotor = hardwareMap.dcMotor.get("left_motor"); // port: 0
        RightMotor = hardwareMap.dcMotor.get("right_motor"); // port: 1

        servo = hardwareMap.servo.get("servo");

        sensor_distance = hardwareMap.get(DistanceSensor.class, "sensor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            if (gamepad1.right_trigger > 0) //forward
            {
                RightMotor.setPower(gamepad1.right_trigger);
                LeftMotor.setPower(-gamepad1.right_trigger);
            }

            else if (gamepad1.left_trigger > 0) //backward
            {
                RightMotor.setPower(-gamepad1.left_trigger);
                LeftMotor.setPower(gamepad1.left_trigger);
            }

            else if (gamepad1.left_stick_x < 0) //turn right
            {
                RightMotor.setPower(0);
                LeftMotor.setPower(-1);
            }

            else if (gamepad1.left_stick_x > 0) //turn left
            {
                RightMotor.setPower(1);
                LeftMotor.setPower(0);
            }

            else if (gamepad1.dpad_right) //drive right
            {
                RightMotor.setPower(-0.8);
                LeftMotor.setPower(-0.8);
            }

            else if (gamepad1.dpad_left) // turn left
            {
                RightMotor.setPower(0.8);
                LeftMotor.setPower(0.8);
            }

            else {
                RightMotor.setPower(0);
                LeftMotor.setPower(0);
            }

            if (gamepad1.x) {
                servo.setPosition(0);
                sleep(300);
                servo.setPosition(1);
                sleep(300);
                servo.setPosition(0);
                sleep(300);
                servo.setPosition(1);
                sleep(300);
                servo.setPosition(0.5);
            }

            if (sensor_distance.getDistance(DistanceUnit.CM) < 25){
                RightMotor.setPower(-1);
                LeftMotor.setPower(1);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

