# S81-F7006 Panel Modbus Client for Windows

A desktop commissioning tool for this communication path:

`Windows PC (Modbus TCP client) -> Moxa MGate MB3170 -> S81-F7006 (Modbus RTU slave)`

The implementation follows **ST-007-EN-R1V1 Modbus RTU Protocol**.

## Features

- Moxa IP address, TCP port, RTU Unit ID, and timeout settings
- FC01: read S81 virtual input and system-input variables (`V-IN` / `V-INSYS`)
- FC02: read S81 virtual output/status variables (`V-OUT`)
- Separate **Panel status LEDs** tab with 37 predefined FC02 panel statuses
- Continuous start/stop monitoring with selectable 0.5, 1, 2, or 5 second polling
- LED state, last-update time, status-change logging, and communication-error indication
- FC05: write a single S81 input or system-input variable
- Named commands: Silence Panel, Silence Sounder, Reset Panel, and Evacuate
- Standard addressing (`SW1-8 OFF`) and extended addressing (`SW1-8 ON`)
- Calculated V-IN, V-OUT, and V-INSYS base addresses
- Four dedicated momentary command push buttons
- FC05 ON (`FF00`) while pressed and FC05 OFF (`0000`) when released
- One-time arming confirmation before momentary commands are enabled
- OFF requests are queued even when a button is clicked very quickly
- Escape-key release and best-effort OFF recovery after communication errors
- Timestamped communication log

## Run on Windows

### Option A - Run using Python

1. Install Python 3.10 or newer from <https://www.python.org/downloads/windows/>.
2. During installation, select **Add Python to PATH**.
3. Extract this project and double-click `run_windows.bat`.

The application itself uses only the Python standard library.

### Option B - Build a standalone EXE

1. Complete steps 1 and 2 above.
2. Double-click `build_windows.bat` while connected to the internet.
3. The executable will be created as `dist\ModbusTCP-Coil-Client.exe`.

The generated EXE can be copied to another Windows PC without installing Python.

## Moxa and S81 configuration

- Enter the Moxa MB3170 Ethernet address in **Moxa IP address**.
- The default Modbus TCP port is `502`.
- Enter the S81-F7006 address selected by `SW1-1` through `SW1-7` as **RTU Unit ID**.
- Configure the Moxa serial port as RTU Slave mode (RTU slaves connected downstream).
- Moxa baud rate, parity, stop bits, RS-485 mode, and wiring must match the S81 card.
- The S81-F7006 must be in slave mode (`SW2-7 OFF`).

## Addressing from ST-007

### Panel status reference conversion

The monitoring list uses conventional Modbus discrete-input references. The
application converts them to the zero-based address sent in the FC02 request:

| Displayed reference | FC02 protocol offset |
| --- | --- |
| 10001 | 0 |
| 10002 | 1 |
| 10047 | 46 |

The monitoring thread reads one block from reference **10001 through 10047**
(wire offset 0, quantity 47) and displays only the 37 defined status points.
Unused gaps are read but not displayed.

### Standard addressing - SW1-8 OFF

| Data area | Addresses |
| --- | --- |
| V-IN using FC01/FC05/FC15 | 0-511 |
| V-INSYS using FC01/FC05/FC15 | 512-543 |
| V-OUT using FC02 | 0-511 |

System command addresses are 512 Silence Panel, 513 Silence Sounder, 514 Reset
Panel, and 515 Evacuate.

### Extended addressing - SW1-8 ON

The offset is `(card address - 1) x 2000`. The application calculates the
appropriate bases and named command addresses. ST-007 limits the card address to
1 through 5 in extended mode.

## Momentary command operation

ST-007 states that each named system command executes when its V-INSYS variable
equals 1. In the **Panel commands** tab:

- Select the addressing mode and, for extended mode, the card SW1 address.
- Select **Arm momentary panel commands** and accept the safety confirmation.
- Press and hold a named command button to write `FF00` (value 1).
- Release the button to write `0000` (value 0).
- Press `Esc` to release an active command.

If a communication error occurs during a held command, the application tries to
open a new TCP connection and confirm an OFF write. Treat any displayed warning
that OFF could not be confirmed as an immediate field-safety condition.

## Panel status LED monitoring

1. Connect to the Moxa MB3170 using the required panel RTU Unit ID.
2. Open the **Panel status LEDs** tab.
3. Select the polling interval and click **Start monitoring**.
4. Each row shows the FC02 reference, the actual zero-based wire offset, and the
   current ON/OFF state.
5. Click **Stop monitoring** before changing communication settings.

LED colors follow the signal type: alarms and evacuation are red, prealarm,
supervisory, tamper and disabled indications are orange, faults are yellow,
normal status/activation indications are green or blue, and inactive/no-data
states are gray. All LEDs change to orange with `COMM ERROR` if polling fails.
After a polling error the client disconnects to prevent a delayed or misaligned
Modbus response from being reused; reconnect before restarting monitoring.
The communication log records monitoring start/stop, the initial active count,
status changes, and errors without logging every successful polling cycle.

## Safety

Reset and Evacuate can directly affect the fire panel and connected equipment.
The software asks for confirmation before every write, but the operator remains
responsible for authorization, isolations, and safe test conditions.
