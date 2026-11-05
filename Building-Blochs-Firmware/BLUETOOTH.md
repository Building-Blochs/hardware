# Connecting to the Bloch Sphere

## Bluetooth connection

The device accepts commands over Bluetooth Low Energy (BLE), using a Nordic UART-style service.

- **Advertised name:** `Bloch-<number>` (for example, `Bloch-1`). The number is configurable on the device.
- **Service UUID:** `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- **Write characteristic (client → device):** `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- **Notify characteristic (device → client):** `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

Use a BLE serial terminal such as nRF Connect, or the project’s `index.html` controller in a browser that supports Web Bluetooth. Scan for the device’s `Bloch-<number>` name, connect, and enable notifications on the TX characteristic to receive replies.

Send UTF-8 text terminated by a newline (`\n`). Commands are case-sensitive. A line may contain one command or multiple comma-separated commands:

    Xgate\n
    led,255,0,0\n
    Xgate,Ygate,reset\n

The device echoes each processed command token on the notify characteristic. `query` also sends the current state as `state,x,y,z`.

## Commands

### Rotations and gates

Gate and rotation angles are about the named axis. For parameterized rotations, `turns` means full revolutions: `0.5` is 180°, `0.25` is 90°, and negative values rotate in the opposite direction.

| Command | Effect |
| --- | --- |
| `Xgate` | Rotate 180° about X |
| `Ygate` | Rotate 180° about Y |
| `Zgate` | Rotate 180° about Z |
| `Hgate` | Rotate 180° about the (X + Z) axis |
| `Sgate` / `S+gate` | Rotate +90° / -90° about Z |
| `Tgate` / `T+gate` | Rotate +45° / -45° about Z |
| `SXgate` | Rotate +90° about X |
| `Rx,<turns>` | Rotate about X by the specified number of turns |
| `Ry,<turns>` | Rotate about Y by the specified number of turns |
| `Rz,<turns>` | Rotate about Z by the specified number of turns |
| `rotate,<ax>,<ay>,<az>,<turns>` | Rotate about the axis vector `(ax, ay, az)` by the specified number of turns |

Examples:

    Rx,0.5
    Rz,-0.25
    rotate,1,1,0,0.125

### Directional moves

These legacy command names are also accepted:

| Command | Effect |
| --- | --- |
| `voorwaarts` / `achterwaarts` | +180° / -180° about X |
| `links` / `rechts` | +180° / -180° about Y |
| `linksboven` / `rechtsboven` | +180° / -180° about the (X + Z) axis |
| `linksonder` | +180° about the (X - Z) axis |

### States and reset

| Command | Effect |
| --- | --- |
| `reset` or `state0` | Return to the initial state by undoing the tracked rotations |
| `home` | Immediately set the software state to the initial state and clear queued motion |
| `state1` | Reset, then rotate 180° about X |
| `statePlus` / `stateMinus` | Reset, then rotate +90° / -90° about Y |
| `stateR` / `stateL` | Reset, then rotate -90° / +90° about X |
| `query` | Notify the current Bloch vector as `state,<x>,<y>,<z>` |

### Display and timing

| Command | Arguments | Effect |
| --- | --- | --- |
| `led` | `led,<r>,<g>,<b>` | Set the screen color |
| `pulse` | `pulse,<r>,<g>,<b>,<milliseconds>` | Set the screen color temporarily, then restore it |
| `task` | `task,<text>` | Set the displayed BLE task text |
| `brightness` | `brightness,<value>` | Set display brightness (byte range 0–255) |
| `delay` | `delay,<milliseconds>` | Pause command processing for the specified time |

Example:

    led,0,100,255
    pulse,255,0,0,500
    task,Running
    brightness,180

## Notes

- Separate commands with commas and terminate the line with a newline.
- Keep command names and arguments in the documented order; commands are case-sensitive.
- BLE commands are ignored while the device is performing calibration.
- The device echoes command tokens, including unrecognized tokens; an echo is not confirmation that a command was valid.