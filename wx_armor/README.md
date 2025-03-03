## Development Guide

1. Start the development environment

   ```bash
   $ nix develop
   ```

2. Build the project

   ```bash
   $ cd wx_armor
   $ mkdir build
   $ cmake ..
   $ make
   ```

3. Run the built binary

   - Make sure the robotic ARM is connected via USB cable
   - Run

     ```bash
     $ cd build
     $ ./wx_armor
     ```

     And the server will be up and running.

4. Connect to the websocket server and issue command. Below is an example using `websocat`.

   ```bash
   $ websocat ws://<ip>:<port>/api/engage
   ```

   You can issue command like `SUBSCRIBE`, `SETPOS [<the joint positions>]`, `TORQUE ON`, `TORQUE OFF`, and `SETPID`.

   The command `SETPID` is in the format of

   ```console
   # Set only the "waist" motor's PID gains
   SETPID [{"name": "waist", "p": 800, "i": 0, "d": 3}]

   # Set all motor's PID gains, but "wrist_rotate" will have different value
   SETPID [{"name": "all", "p": 800, "i": 0, "d": 3}, {"name": "wrist_rotate", "p": 400, "i": 0, "d": 0}]
   ```

## Deployment Guide

This service is normally deployed with systemd on a server. As long as you know the server ip and port, you can connect to the websocket service via the ip and port. Everything should work as expected.

### How to deploy a new version of this to Unitree onboard Raspberry Pi?

1. Update [robot-system-config](https://github.com/HorizonRoboticsInternal/robot-system-config/blob/main/flake.nix) to use the newest commit of `interbotix-xs-driver`.

2. Follow the instruction of [robot-system-config](https://github.com/HorizonRoboticsInternal/robot-system-config) to deploy the config to a raspberry Pi.

3. Now that Pi will be running the newest version of `wx_armor`.
