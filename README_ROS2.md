# 4-DOF Arm: ESP32 Code and ROS 2 Simulation (Windows 10)

Files in this folder:

| File | What it does |
|---|---|
| `arm_joystick.ino` | ESP32 firmware: joystick moves the end effector in X-Y, 2 buttons move it up / down, and angles are printed over USB |
| `arm.urdf` | 3D model of the arm (4 revolute joints) for RViz |
| `arm_serial_bridge.py` | Reads angles from the ESP32 and publishes `/joint_states` |
| `arm_view.launch.py` | Starts everything: state publisher, RViz, bridge |

## Part 1: Upload the ESP32 code

1. Install the Arduino IDE. In *Boards Manager* install **esp32 by Espressif**.
2. In *Library Manager* install **ESP32Servo**.
3. Open `arm_joystick.ino`, select **ESP32 Dev Module** and your COM port, then upload.
4. Open Serial Monitor at **115200**. You should see lines like `A,90.0,121.9,162.9,165.2` (the arm starts at its home pose).
5. Before powering on, leave the joystick untouched, since it calibrates its centre at startup, and keep the arm area clear because it moves to home at power-on.
6. If a joint moves the wrong way, set its entry in `SIGN[]` to `-1`. If the stick moves the arm the wrong way, flip `INVERT_X` / `INVERT_Y`.

**Controls (end-effector control):**

| Input | Action |
|---|---|
| Joystick forward / back | End effector moves forward / back |
| Joystick left / right | End effector moves left / right |
| Button 1 (hold) | End effector moves up |
| Button 2 (hold) | End effector moves down |
| Both buttons | Return to home position |
| Onboard LED on | Target is out of reach, the arm stays where it was |

The ESP32 solves the inverse kinematics, so the link lengths at the top of the `.ino` file (`L1`, `L2`, `L3`, `Z_SHOULDER`) must match your real arm and `arm.urdf`.

Close the Arduino Serial Monitor before using ROS 2, because only one program can use the COM port.

## Part 2: See the arm in ROS 2 (RViz)

1. Install ROS 2 on Windows 10 using the official Windows binary instructions at docs.ros.org (pick a distro that lists Windows 10 support).
2. Open the ROS 2 command prompt (the one that runs `call ...\local_setup.bat` for you).
3. Install the serial library: `pip install pyserial`
4. Go to this folder: `cd path\to\this\folder`
5. **No hardware yet?** Test with keyboard control first:
   - In this window run `ros2 launch arm_view.launch.py demo:=true` (opens RViz only).
   - Open a **second** ROS 2 command prompt (load ROS 2 again, `cd` to the same folder) and run
     `python arm_serial_bridge.py --demo`
   - The arm holds still at the home pose. Click the second window and press **W/S** forward/back, **A/D** left/right, **R/F** up/down, **H** home, **Q** quit.
6. **With the ESP32 connected** (find the port in Device Manager, e.g. COM5):
   `ros2 launch arm_view.launch.py port:=COM5`
7. In RViz, do these two things once:
   - Set **Global Options > Fixed Frame** to `base_link`
   - Click **Add > RobotModel**, then set its *Description Source* to Topic and Description Topic to `/robot_description`

The model in RViz should now follow your joystick.

## Troubleshooting

- **Model is all white or missing:** the Fixed Frame is wrong, or RobotModel was not added.
- **`could not open port`:** wrong COM number, or the Arduino Serial Monitor is still open.
- **Arm in RViz moves opposite to the real arm:** in `arm_serial_bridge.py`, negate the angle for that joint.
- **Base keeps turning on the real arm with no input:** the base is a continuous-rotation servo. It cannot hold a position, so replace it with a normal positional servo.
- **Real arm links are not 16 cm / 14 cm:** edit the lengths in `arm.urdf` to match your build.

## Next steps

- Add a gripper: a 5th servo, plus a revolute or prismatic joint in `arm.urdf`.
- Move to MoveIt 2 for planned motion once the model matches the real arm.
