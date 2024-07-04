# Core

## Components
1. Slaveboard: BluePill-based board that will control motors and actuators.
2. Robot: ESP32-based board that will direct the robot

## Adding a new component

### Setup

1. Create a new folder in `src`, such as `new_board`, for the board-specific source code, and header files in `include` if necessary. 
2. Add board-agnostic code in `src/common` and `include/common`. Be careful, as functions such as `pwm_start` are for BluePill, not ESP32. 
3. Create a new environment in `platformio.ini` such as `[env:new_board]`. Define a build flag with `build_flags = -D NEW_BOARD`. 
4. Specify what folders should be compiled to be flashed. If you need `common` and your `new_board`, add `build_src_filter = +<common/*> +<new_board/*>` to the environment.
5. Wrap all board-specific code and declarations in the build flag you created:
```cpp
#ifdef NEW_BOARD

// your code here

#endif
```

### Compiling and Uploading
1. If you're using VSCode, ensure to change the active environment by clicking `Default (core)`, or whatever environment is active, on the bottom taskbar and select your new environment. Now, you can compile and upload as usual.
2. Otherwise, or optionally, run `pio run -e new_board -t upload` to compile and upload. Similarly, run `pio monitor -e new_board` to activate the serial monitor for your new board.