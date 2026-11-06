# Wireless firmware updates

## First installation

Build and upload `m5stack-cores3` over USB using PlatformIO. The normal USB
upload must include the bootloader and partition table, not just the application.
The explicitly selected `default_16MB.csv` layout has two 6.25 MiB app slots and
preserves the board's existing default layout and settings partition.

Host `update.html` on HTTPS (localhost is also suitable for development).
Open it in Chrome or Edge on a Bluetooth-enabled PC. Connect a `Bloch-` device,
select a signed `.bbu` package, and install over Bluetooth, or use the Wi-Fi
workflow below. Bluetooth file transfer does not require Wi-Fi credentials.
The device must be idle and out of calibration; controls are suspended during
transfer. Normal startup movement occurs after reboot. Reconnect afterward to
read the installed version. If transfer fails, try slower compatibility mode.

## Wi-Fi setup and updates

Install a firmware version with Wi-Fi support once using the existing Bluetooth
updater or USB. In `update.html`, connect the device, enter its 2.4 GHz network
name and password, and choose **Save and connect Wi-Fi**. Credentials are saved
in a separate NVS `wifi` namespace and reused after reboot. Network names support
1–32 UTF-8 bytes; passwords support 8–63 bytes, or empty for an open network.
Commas and spaces are preserved. The device must be idle when saving credentials.

The top-right Wi-Fi symbol shows a green check when the internet check passes,
and an amber cross otherwise. It remains above the CONNECTED screen, settings,
and update screen. Internet access is checked with an HTTP 204 request to
`http://connectivitycheck.gstatic.com/generate_204` every 30 seconds while idle.
A router connection alone does not produce a green check. A network blocking
that endpoint may show a cross even though a particular update host is reachable.
Downloads require a router connection, not a passing internet check, so a local
HTTP server can also be used.

Paste a direct HTTP or HTTPS `.bbu` URL and choose **Install over Wi-Fi**. The
browser only sends the URL; the device downloads and verifies the signed file.
The server must return HTTP 200 with the file, without authentication or redirects.
Content-Length, chunked, and connection-delimited responses are supported. Links
can contain up to 1024 UTF-8 bytes. The page reports downloaded bytes because
chunked responses may not advertise a total size. Cancel works while Bluetooth
is connected; a Bluetooth disconnect does not interrupt a Wi-Fi update. Keep
power on and reconnect afterward to read the installed version.

Network operations run in a separate task with bounded buffering and timeouts.
The main task owns flash writes and all package verification. Motion and touch
controls are suspended while downloading/installing, while the status icon and
update screen remain live. A failed or interrupted download leaves the current
boot partition selected. Wi-Fi reconnects automatically when the network returns.

As with the existing updater, BLE is not paired: provision credentials only in a
trusted environment. Credentials are stored in ordinary device NVS and are never
returned over BLE or sent to the download host. HTTPS downloads use encryption
without certificate validation; firmware authenticity relies on the required BBU
publisher signature and image hash, for both HTTP and HTTPS. Do not use URLs
containing secrets that require authenticated TLS. The internet probe is a
connectivity indicator, not a security guarantee.

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
the `.bbu` file and the updater page. The page supports local file selection for
Bluetooth and a direct download URL for Wi-Fi; it does not discover releases. Firmware version is derived
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

A Bluetooth-transfer disconnect, cancellation, malformed package, or 60-second
inactivity timeout aborts the transfer. Wi-Fi body reads also have a 15-second
idle timeout; Wi-Fi downloads continue after Bluetooth disconnects. Restart the whole transfer after reconnecting. The active
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
`a0741002` (read status), `a0741003` (read board/version), and
`a0741004` (read one-byte Wi-Fi state). Wi-Fi states: 0 unconfigured, 1 connecting,
2 associated but internet check pending/failed, 3 internet available, 4 disconnected.
Older firmware has no `a0741004`; the page disables Wi-Fi features in that case.

Commands: byte `1` begins; `2` followed by a little-endian uint32 package offset
and up to 239 payload bytes transfers data; `3` cancels; `4` verifies and commits.
Do not cross the 300-byte package-header boundary in a single packet. Only one
packet may be outstanding. Read status until its acknowledged offset advances.
Duplicate/out-of-order packets return the current offset without writing again.

Status is five bytes: code and little-endian uint32 received offset. Codes:
0 idle/cancelled; 1 receiving; 2 committed/restarting; 3 busy or trial boot;
4 invalid package; 5 flash/activation error; 6 invalid command; 7 timeout;
8 staged command acknowledged; 9 credentials saved; 10 Wi-Fi unavailable;
11 download/HTTP failure; 12 invalid credentials or persistence failure;
13 command queued (keep polling).

Wi-Fi commands use the same write characteristic and five-byte status:

1. Write `[0x10]` to clear the staging buffer; wait for code 8, offset 0.
2. Write `[0x11, offset_lo, offset_hi, ...payload]` to append bytes. Wait for code
   8 and the new staged byte count after every fragment. Duplicate/out-of-order
   offsets return the current count without appending. Maximum payload: 1024 bytes.
   Use at most 17 payload bytes per fragment for the minimum ATT MTU.
3. For credentials, stage `[ssid_byte_length, ...ssid_utf8, ...password_utf8]`,
   then write `[0x12]`. Code 9 means saved, not yet associated; poll Wi-Fi state.
4. For a download, stage the UTF-8 URL without a terminator, then write `[0x13]`.
   Code 1 means accepted; the status offset now counts received BBU bytes.
   Poll until code 2 (verified, committed, restarting) or an error.
5. `[3]` cancels either transfer and clears staging. Staging is also cleared on
   disconnect and after 60 seconds without a command. Starts/configuration are
   rejected during movement, calibration, another network job, or a trial boot.

The device still verifies the signed 300-byte header before opening the OTA slot,
then verifies the full image length and checksum before committing. Wi-Fi blocks
may span the header boundary; the Bluetooth data command retains its original
boundary requirement.
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

- Save Wi-Fi credentials containing spaces/commas, reboot, and verify reconnection.
- Test incorrect credentials, lost access point, and a captive portal. Check the
  icon on normal, CONNECTED, settings, and update screens.
- Install via HTTP and HTTPS; test Content-Length and chunked responses, 404,
  redirects, truncated/oversized/corrupt files, stalls, cancellation, and Wi-Fi loss.
- Disconnect Bluetooth during Wi-Fi download and confirm installation completes.
- Verify an older firmware still installs a local file from the updated page.

Host-side updater checks: `node --test test/test_update_page.cjs` and
`python -m unittest discover -s test -p test_update_package.py`.

These physical checks cannot be replaced by a successful compilation.
