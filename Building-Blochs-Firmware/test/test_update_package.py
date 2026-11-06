"""Host-side release envelope checks: python -m unittest discover -s test -p test_update_package.py"""
import hashlib
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding

ROOT = Path(__file__).resolve().parents[1]

class UpdatePackageTests(unittest.TestCase):
    def test_signed_package_and_tamper_detection(self):
        with tempfile.TemporaryDirectory() as directory:
            image = bytearray(512)
            image[0] = 0xE9
            image[12] = 9
            source = Path(directory) / 'firmware.bin'
            output = Path(directory) / 'update.bbu'
            source.write_bytes(image)
            subprocess.run([sys.executable, str(ROOT/'scripts/package_update.py'),
                'pack', str(source), str(output)], check=True, capture_output=True)
            package = output.read_bytes()
            self.assertEqual(package[:8], b'BBOTA001')
            self.assertEqual(struct.unpack_from('<I', package, 8)[0], len(image))
            self.assertEqual(package[12:44], hashlib.sha256(image).digest())
            public_text = (ROOT/'include/update_public_key.h').read_text().split('R"KEY(')[1].split(')KEY"')[0]
            public = serialization.load_pem_public_key(public_text.encode())
            public.verify(package[44:300], package[:44], padding.PKCS1v15(), hashes.SHA256())
            changed = bytearray(package[:44]); changed[8] ^= 1
            with self.assertRaises(InvalidSignature):
                public.verify(package[44:300], bytes(changed), padding.PKCS1v15(), hashes.SHA256())
            corrupt = bytearray(package[300:]); corrupt[-1] ^= 1
            self.assertNotEqual(package[12:44], hashlib.sha256(corrupt).digest())

    def test_reject_wrong_chip(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'wrong.bin'
            source.write_bytes(bytes([0xE9])+bytes(511))
            result = subprocess.run([sys.executable, str(ROOT/'scripts/package_update.py'),
                'pack', str(source), str(Path(directory)/'out.bbu')], capture_output=True)
            self.assertNotEqual(result.returncode, 0)

if __name__ == '__main__':
    unittest.main()
