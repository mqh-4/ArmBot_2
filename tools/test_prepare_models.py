"""Model installation checks: no network or ROS/hardware access."""
import hashlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "prepare_models", Path(__file__).with_name("prepare_models.py"),
)
models = importlib.util.module_from_spec(spec)
spec.loader.exec_module(models)


class ModelInstallTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.target = Path(self.folder.name) / "model.bin"
        self.payload = b"verified-model-payload"
        self.entry = {
            "size": len(self.payload),
            "sha256": hashlib.sha256(self.payload).hexdigest(),
            "source": "example/resolve/main/model.bin",
        }

    def test_success_installs_verified_payload(self):
        with patch.object(models.urllib.request, "urlopen",
                          return_value=io.BytesIO(self.payload)):
            models.download(self.target, self.entry, "https://huggingface.co")
        self.assertEqual(self.target.read_bytes(), self.payload)
        self.assertEqual(list(self.target.parent.glob("*.part")), [])

    def test_checksum_failure_does_not_install_payload(self):
        wrong_payload = b"x" * len(self.payload)
        with patch.object(models.urllib.request, "urlopen",
                          return_value=io.BytesIO(wrong_payload)):
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                models.download(self.target, self.entry, "https://huggingface.co")
        self.assertFalse(self.target.exists())
        self.assertEqual(list(self.target.parent.glob("*.part")), [])

    def test_size_failure_does_not_install_payload(self):
        with patch.object(models.urllib.request, "urlopen",
                          return_value=io.BytesIO(self.payload + b"extra")):
            with self.assertRaises(ValueError):
                models.download(self.target, self.entry, "https://huggingface.co")
        self.assertFalse(self.target.exists())
        self.assertEqual(list(self.target.parent.glob("*.part")), [])

    def test_existing_corrupt_file_is_preserved_without_network(self):
        self.target.write_bytes(b"do-not-overwrite")
        with patch.object(models.urllib.request, "urlopen") as request:
            with self.assertRaises(ValueError):
                models.download(self.target, self.entry, "https://huggingface.co")
        self.assertEqual(self.target.read_bytes(), b"do-not-overwrite")
        request.assert_not_called()


if __name__ == "__main__":
    unittest.main()
