"""Create a signing identity once, then package application firmware for BLE OTA.

python scripts/package_update.py init
python scripts/package_update.py pack .pio/build/m5stack-cores3/firmware.bin release/update.bbu
Requires: pip install cryptography. Back up .update-keys/private.pem securely.
"""
import argparse
import hashlib
import struct
from pathlib import Path
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa

ROOT = Path(__file__).resolve().parents[1]
KEY = ROOT / '.update-keys/private.pem'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['init', 'pack'])
    parser.add_argument('input', nargs='?')
    parser.add_argument('output', nargs='?')
    args = parser.parse_args()
    if args.action == 'init':
        if KEY.exists() or (ROOT / 'include/update_public_key.h').exists():
            parser.error('Signing identity already exists; refusing to replace it.')
        key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        KEY.parent.mkdir(exist_ok=True)
        KEY.write_bytes(key.private_bytes(serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
        public = key.public_key().public_bytes(serialization.Encoding.PEM,
            serialization.PublicFormat.SubjectPublicKeyInfo).decode()
        (ROOT / 'include/update_public_key.h').write_text(
            '#pragma once\nstatic const char UPDATE_PUBLIC_KEY[] = R"KEY(' + public + ')KEY";\n')
        print('Signing identity created. Back up the private key; never publish it.')
        return
    if not args.input or not args.output:
        parser.error('pack requires input firmware.bin and output update.bbu')
    firmware = Path(args.input).read_bytes()
    # ESP image header: magic and ESP32-S3 chip ID. Package only the application.
    if len(firmware) < 24 or firmware[0] != 0xE9 or struct.unpack_from('<H', firmware, 12)[0] != 9:
        parser.error('Expected an ESP32-S3 application image')
    if len(firmware) > 0x640000:
        parser.error('Firmware exceeds OTA slot size')
    header = b'BBOTA001' + struct.pack('<I', len(firmware)) + hashlib.sha256(firmware).digest()
    key = serialization.load_pem_private_key(KEY.read_bytes(), password=None)
    signature = key.sign(header, padding.PKCS1v15(), hashes.SHA256())
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(header + signature + firmware)
    print(f'Wrote signed update: {output} ({len(firmware)} firmware bytes)')

if __name__ == '__main__':
    main()
