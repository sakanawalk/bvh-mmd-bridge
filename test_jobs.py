import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from jobs import run_conversion

class JobTests(unittest.TestCase):
    def test_metadata_failure_does_not_publish_vmd(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            def convert(*args):Path(args[2]).write_bytes(b'partial');return {'frames':1}
            with patch('jobs.convert',side_effect=convert),patch('jobs.json.dumps',side_effect=OSError('disk full')):
                with self.assertRaises(OSError):run_conversion('source','model',folder,{})
            self.assertFalse((folder/'motion.vmd').exists())
            self.assertTrue((folder/'motion.partial.vmd').exists())
    def test_existing_result_is_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);(folder/'motion.vmd').write_bytes(b'keep')
            with self.assertRaises(FileExistsError):run_conversion('source','model',folder,{})
            self.assertEqual((folder/'motion.vmd').read_bytes(),b'keep')
