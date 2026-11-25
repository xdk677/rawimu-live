from pathlib import Path
import tempfile
import unittest

import numpy as np
from test_rawimu_live import ROOT, live, session


def gp_line(stamp=100., count=13, gyro=(6553600, -6553600, -4325376),
            acc=(0, 0, 128537723), temperature=3145, week=2446):
    fields = ['GPIMU', '%.9f' % stamp, str(count)]
    fields += ['%08X' % (v & 0xffffffff) for v in tuple(gyro) + tuple(acc)]
    fields += [str(week), '%.9f' % stamp, '0', '0', '0', '0', '0', ' 1', '%08X' % (temperature & 0xffffffff), '00000000']
    payload = ','.join(fields)
    checksum = 0
    for value in payload.encode('ascii'):
        checksum ^= value
    return ('$' + payload + '*%02X\r\n' % checksum).encode('ascii')


class GPIMUDataTests(unittest.TestCase):
    def test_mixed_fixture_replay_retains_raw_timeline_and_exact_epoch_comparison(self):
        from gpimu_data import convert_rows, match_rawimu
        s = session()
        worker = live.ReplayWorker(s, str(ROOT / 'examples/gpimu_test.dat'), 0)
        worker.run()
        self.assertFalse(s.error, s.error)
        self.assertFalse(worker.index.gpimu_only)
        self.assertEqual(s.n_imu, 1200)
        self.assertEqual(s.gpimu.total, 1200)
        data = s.gpimu_snapshot()
        values, _ = convert_rows(data['rows'], 'M1633')
        matched, offsets = match_rawimu(data['rows'][:, 1], data['raw_rows'])
        self.assertTrue(np.isfinite(offsets).all())
        np.testing.assert_allclose(values, matched, atol=1e-12)
        s.close_analysis()

    def test_unready_gps_uses_imu_clock_and_is_not_time_matched(self):
        from gpimu_data import decode_gpimu, GPIMUStore, match_rawimu
        store = GPIMUStore()
        store.append(decode_gpimu(gp_line(100., week=0)))
        store.append(decode_gpimu(gp_line(100.01, week=0)))
        rows = store.snapshot()
        self.assertTrue(np.isnan(rows[:, 1]).all())
        self.assertAlmostEqual(rows[-1, 13], 10., places=6)
        self.assertEqual(rows[-1, 15], 0)
        matched, _ = match_rawimu(rows[:, 1], np.array([[100., 1, 2, 3, 4, 5, 6]]))
        self.assertTrue(np.isnan(matched).all())

    def test_m22_temperature_uses_signed_low16_bits_like_matlab_combined_model(self):
        from gpimu_data import decode_gpimu, convert_gpimu, convert_rows, GPIMUStore
        raw = decode_gpimu(gp_line(temperature=0xff80))
        self.assertAlmostEqual(convert_gpimu(raw, 'M22')[1], -1.)
        store = GPIMUStore()
        store.append(raw)
        self.assertAlmostEqual(convert_rows(store.snapshot(), 'M22')[1][0], -1.)
        self.assertEqual(raw['temperature_raw'], 0xff80)

    def test_duplicate_gpimu_epochs_are_ambiguous_and_not_compared(self):
        from gpimu_data import match_rawimu
        stamps = np.array([100., 100., 100.01])
        raw_rows = np.array([[100., 1, 2, 3, 4, 5, 6], [100.01, 7, 8, 9, 10, 11, 12]])
        values, offsets = match_rawimu(stamps, raw_rows, .002)
        self.assertTrue(np.isnan(values[:2]).all())
        np.testing.assert_allclose(values[2], [7, 8, 9, 10, 11, 12])

    def test_signed_fields_checksum_count_and_model_specific_scales(self):
        from gpimu_data import decode_gpimu, convert_gpimu
        raw = decode_gpimu(gp_line(temperature=-100))
        self.assertEqual(raw['sample_count'], 13)
        self.assertEqual(raw['gyro'][1], -6553600)
        values, temperature = convert_gpimu(raw, 'M1633')
        np.testing.assert_allclose(values[:2], [1., -1.])
        self.assertAlmostEqual(temperature, -1.)
        values, temperature = convert_gpimu(raw, 'M22')
        self.assertAlmostEqual(values[2], 1.)
        self.assertAlmostEqual(temperature, -100/128.)
        values, temperature = convert_gpimu(raw, 'M21')
        self.assertAlmostEqual(temperature, 25-100/256.)
        corrupted = gp_line().replace(b',13,', b',14,')
        with self.assertRaises(ValueError):
            decode_gpimu(corrupted)

    def test_gpimu_is_auxiliary_and_does_not_double_integrate_rawimu(self):
        raw = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:144]
        s = session()
        s.feed(gp_line() + raw[:72] + gp_line(100.01, count=13) + raw[72:])
        self.assertEqual(s.n_imu, 2)
        self.assertEqual(s.gpimu.total, 2)
        data = s.gpimu_snapshot()
        self.assertEqual(list(data['rows'][:, 5]), [13, 13])
        self.assertAlmostEqual(data['rows'][1, 13], 10., places=3)
        self.assertEqual(len(data['raw_rows']), 2)
        s.close_analysis()

    def test_bad_gpimu_checksum_and_format_are_counted_separately(self):
        s = session()
        s.feed(gp_line().replace(b',13,', b',14,') + b'$GPIMU,garbage*00\r\n' + gp_line())
        self.assertEqual(s.gpimu.total, 1)
        self.assertEqual(s.parser.n_gpimu_crc_bad + s.parser.n_gpimu_malformed, 2)
        self.assertEqual(s.parser.n_crc + s.parser.n_text_crc_bad, 0)
        s.close_analysis()

    def test_nearest_gps_comparison_is_one_to_one_without_interpolation(self):
        from gpimu_data import match_rawimu
        gp_stamps = np.array([100., 100.0001, 100.01, np.nan, 100.1])
        raw_rows = np.array([[100., 1, 2, 3, 4, 5, 6], [100.01, 7, 8, 9, 10, 11, 12]])
        values, delta = match_rawimu(gp_stamps, raw_rows, .002)
        np.testing.assert_allclose(values[0], [1, 2, 3, 4, 5, 6])
        self.assertTrue(np.isnan(values[1]).all())
        np.testing.assert_allclose(values[2], [7, 8, 9, 10, 11, 12])
        self.assertTrue(np.isnan(values[3:]).all())
        self.assertEqual(delta[0], 0)

    def test_gpimu_only_replay_uses_time_axis_without_generating_rawimu(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'gp.dat'
            path.write_bytes(b''.join(gp_line(100 + k*.01, count=13) for k in range(10)))
            s = session()
            worker = live.ReplayWorker(s, str(path), 0)
            worker.run()
            self.assertFalse(s.error, s.error)
            self.assertEqual(s.n_imu, 0)
            self.assertEqual(s.gpimu.total, 10)
            self.assertTrue(worker.index.gpimu_only)
            self.assertAlmostEqual(worker.timeline()['duration'], .09, places=4)
            s.close_analysis()

    def test_independent_file_loader_returns_gp_and_raw_data(self):
        from gpimu_data import load_gpimu_recording
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'mixed.dat'
            path.write_bytes(gp_line() + (ROOT / 'examples/rawimu_test.dat').read_bytes()[:72] + gp_line(100.01))
            data = load_gpimu_recording(path, dict(live.DEFAULT_CFG))
            self.assertEqual(data['rows'].shape, (2, 16))
            self.assertEqual(data['raw_rows'].shape, (1, 7))
            self.assertAlmostEqual(data['rows'][1, 13], 10., places=3)


if __name__ == '__main__':
    unittest.main()
