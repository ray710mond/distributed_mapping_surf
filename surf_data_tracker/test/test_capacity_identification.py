import unittest
import numpy as np
from surf_data_tracker.capacity import usable_capacity
from surf_data_tracker.identification import fit_model

class CapacityIdentificationTests(unittest.TestCase):
    def test_direct_preferred(self):
        live={'morse_sample_valid':True,'s1g_average_throughput_mbps':10,
              's1g_success_probability_pct':100}
        self.assertEqual(usable_capacity(live,
                         measured_application_bps=200,mmrc_factor=.5)[0],200)
        self.assertEqual(usable_capacity({'tx_interface_mbps':100})[0],0)
        self.assertEqual(usable_capacity({'tx_compatibility_rate_mbps':100})[0],0)
        self.assertEqual(usable_capacity({**live,'s1g_average_throughput_mbps':2},mmrc_factor=.5)[0],125000)

    def test_stale_or_missing_radio_state_stops_map_credit(self):
        live={'morse_sample_valid':True,'s1g_average_throughput_mbps':2,
              's1g_success_probability_pct':80}
        self.assertEqual(usable_capacity(live,mmrc_factor=.1),
                         (20000.0,'guarded_mmrc_estimate'))
        self.assertEqual(usable_capacity({**live,'morse_sample_valid':False})[0],0)
        self.assertEqual(usable_capacity({**live,'s1g_success_probability_pct':0})[0],0)

    def test_known_identification(self):
        rng=np.random.default_rng(71)
        x=rng.uniform(0,100,(100,2));u=rng.uniform(0,100,(100,2));d=rng.uniform(0,1,(100,2))
        a=np.array([[.9,.1],[0,.8]]);b=np.array([[-.1,0],[0,-.2]])
        fit=fit_model(x,u,x@a.T+u@b.T+d,d)
        np.testing.assert_allclose(fit['A'],a,atol=1e-12)
        np.testing.assert_allclose(fit['B'],b,atol=1e-12)
        self.assertEqual(fit['rank'],4)

class TemporalMetricTests(unittest.TestCase):
    def test_nonzero_regression_is_a_correctness_failure(self):
        from surf_data_tracker.analysis import derived_measurements
        event={'category':'delivery','pipeline':'mapping','stage':'backlog',
               'payload':{'applied_temporal_regressions':1}}
        with self.assertRaisesRegex(ValueError,'Correctness failure'):
            list(derived_measurements(event))

class AllocationSummaryTests(unittest.TestCase):
    def test_each_metric_appears_once_with_correct_unit(self):
        from surf_data_tracker.analysis import derived_measurements
        event={'category':'allocation','pipeline':'mapping','stage':'controller',
               'payload':{'debt_0':4.5,'allocated_rate_0':100.,'configuration_revision':2,
                          'experimental_capacity_factor':.5}}
        rows=list(derived_measurements(event))
        self.assertEqual(len(rows),4)
        units={r[3]:r[4] for r in rows}
        self.assertEqual(units['debt_0'],'priority')
        self.assertEqual(units['allocated_rate_0'],'bytes/s')
        self.assertEqual(units['experimental_capacity_factor'],'dimensionless')

class InformationPacketLossTests(unittest.TestCase):
    def test_interleaved_traffic_is_not_loss(self):
        from surf_data_tracker.analysis import enrich_delivery
        events=[{'category':'delivery','wall_time_ns':i,'payload':{
            'protocol':'information','source_id':'drone','map_epoch':1,
            'traffic_class':stream,'version':i,'chunk_count':1,'chunk_index':0}}
            for i,stream in [(1,1),(2,2),(3,1),(5,2)]]
        enrich_delivery(events)
        self.assertEqual([e['payload']['sequence_gap_updates'] for e in events],[0,0,0,1])
