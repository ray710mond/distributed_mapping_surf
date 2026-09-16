import unittest
import numpy as np
from surf_data_tracker.capacity import usable_capacity
from surf_data_tracker.identification import (
    acknowledged_service_diagnostics, fit_delayed_model, fit_model)

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

    def test_delayed_identification_separates_input_lags(self):
        rng=np.random.default_rng(72)
        count=500
        x=rng.uniform(10,100,(count,2))
        history=rng.uniform(0,100,(count,3,2))
        a=np.array([[.95,.02],[.01,.9]])
        delayed_b=np.array([[-.08,0],[0,-.12]])
        y=x@a.T+history[:,2,:]@delayed_b.T
        fit=fit_delayed_model(x,history,y,np.zeros_like(y))
        np.testing.assert_allclose(fit['A'],a,atol=1e-12)
        np.testing.assert_allclose(fit['B_lags'][0],np.zeros((2,2)),atol=1e-12)
        np.testing.assert_allclose(fit['B_lags'][1],np.zeros((2,2)),atol=1e-12)
        np.testing.assert_allclose(fit['B_lags'][2],delayed_b,atol=1e-12)

    def test_acknowledged_service_closes_state_accounting(self):
        rows=[]
        debt=np.array([10.,20.])
        disturbance=np.zeros(2)
        acknowledged=np.zeros(2)
        for step,(change,ack) in enumerate((([2.,1.],[0.,0.]),([0.,3.],[4.,2.]))):
            rows.append({'debt_0':debt[0],'debt_1':debt[1],
                         'disturbance_0':disturbance[0],'disturbance_1':disturbance[1],
                         'acknowledged_debt_0':acknowledged[0],
                         'acknowledged_debt_1':acknowledged[1],
                         'wire_bytes_0':100,'wire_bytes_1':200})
            disturbance+=change
            acknowledged+=ack
            debt+=np.array(change)-np.array(ack)
        rows.append({'debt_0':debt[0],'debt_1':debt[1],
                     'disturbance_0':disturbance[0],'disturbance_1':disturbance[1],
                     'acknowledged_debt_0':acknowledged[0],
                     'acknowledged_debt_1':acknowledged[1],
                     'wire_bytes_0':0,'wire_bytes_1':0})
        result=acknowledged_service_diagnostics([rows])
        np.testing.assert_allclose(result['state_accounting_rmse'],[0,0])
        np.testing.assert_allclose(result['acknowledged_debt'],[4,2])

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
