# Bluetooth firmware updates

## First installation

Build and upload `m5stack-cores3` over USB using PlatformIO. The normal USB
upload must include the bootloader and partition table, not just the application.
The explicitly selected `default_16MB.csv` layout has two 6.25 MiB app slots and
preserves the board's existing default layout and settings partition.

Host `update.html` on HTTPS (localhost is also suitable for development).
Open it in Chrome or Edge on a Bluetooth-enabled PC. Connect a `Bloch-` device,
select a signed `.bbu` package, and install. No Wi-Fi credentials are required.
The device must be idle and out of calibration; controls are suspended during
transfer. Normal startup movement occurs after reboot. Reconnect afterward to
read the installed version. If transfer fails, try slower compatibility mode.

## Publishing a release

The repository contains the public verification key. The matching private key
was generated locally in `.update-keys/private.pem`, which is excluded from Git.
Back it up securely. Never serve it with the website or distribute it with a release.
Future maintainers need that key to sign updates accepted by these devices.
Do not regenerate the identity for each release. Replacing the trusted key requires
an intentional migration signed by the old key, or USB installation.

```
pio run -e m5stack-cores3
python scripts/package_update.py pack .pio/build/m5stack-cores3/firmware.bin release/update.bbu
```

The packaging script requires Python's `cryptography` package. Distribute only
the `.bbu` file and the updater page. The page currently uses local file selection;
it does not discover releases from a hosting service. Firmware version is derived
from Git by the existing build script; tag releases before building.

## Integrity and recovery

Packages contain a board/protocol marker, image length, SHA-256 hash, RSA-2048
PKCS#1 v1.5 signature over those fields, and the application image. The device
verifies the signature before erasing the inactive slot and verifies the image
hash and ESP image format before selecting the new boot slot. ESP-IDF checks chip
compatibility when validating the image. This is application-level signature
verification, not eFuse secure boot or anti-downgrade protection. Older correctly
signed releases are permitted. Nearby clients can connect and interrupt an update;
there is no physical authorization or BLE pairing requirement in this version.

A disconnect, cancellation, malformed package, or 60-second inactivity timeout
aborts the transfer. Restart the whole transfer after reconnecting. The active
image remains untouched until the new image has been validated.

The board's bundled Arduino/ESP-IDF libraries enable bootloader rollback.
The application overrides Arduino's automatic early acceptance. A trial image
is accepted after ten seconds of startup when the motor driver initialized and
the motor task heartbeat is fresh. A 30-second startup deadline restarts an
unconfirmed image; a reset before confirmation lets the bootloader roll back.
This checks basic execution, not physical motor accuracy, screen quality, or all
application behavior. USB remains the recovery path for faults after acceptance,
bootloader/partition changes, or unavailable valid firmware slots.

## BLE protocol

Service: `a0741000-7d16-4a22-b123-97fe36c04200`.
Characteristics use the same suffix with prefixes `a0741001` (write),
`a0741002` (read status), and `a0741003` (read board/version).

Commands: byte `1` begins; `2` followed by a little-endian uint32 package offset
and up to 239 payload bytes transfers data; `3` cancels; `4` verifies and commits.
Do not cross the 300-byte package-header boundary in a single packet. Only one
packet may be outstanding. Read status until its acknowledged offset advances.
Duplicate/out-of-order packets return the current offset without writing again.

Status is five bytes: code and little-endian uint32 received offset. Codes:
0 idle/cancelled; 1 receiving; 2 committed/restarting; 3 busy or trial boot;
4 invalid package; 5 flash/activation error; 6 invalid command; 7 timeout.
The browser uses 180-byte payloads by default and 15-byte payloads in compatibility
mode for hosts limited to the minimum ATT MTU.

## Hardware acceptance checks before deployment

- Install over USB, exercise all three motors, and read the version through BLE.
- Install a signed release; reconnect and verify the new version and normal motion.
- Disconnect, cancel, and power-cycle during header and image transfers; the old
  application must still boot and allow another update.
- Try corrupted and wrong-key packages; neither may be activated.
- Attempt updates during motion/calibration and commands during transfer.
- Test a deliberately failing trial image on a bench device and verify rollback.
- Check calibration/settings survive updates and that both OTA slots work by
  performing two consecutive releases.

These physical checks cannot be replaced by a successful compilation.
