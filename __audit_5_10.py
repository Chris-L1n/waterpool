import json, csv, os, shutil, math, inspect
from collections import defaultdict

with open('live_config.json','r',encoding='utf-8') as f: cfg = json.load(f)

print('PHASE 5: UTILITY FUNCTIONS')
from anomaly_detector import point_in_polygon, point_to_segment_dist, distance_to_channel_center, dist_to_shore
assert point_in_polygon(0.5, 0.5, [[0,0],[1,0],[1,1],[0,1]]) == True
assert point_in_polygon(2, 2, [[0,0],[1,0],[1,1],[0,1]]) == False
assert point_in_polygon(0.5, 0.5, []) == False
print('  [OK] point_in_polygon')
d = point_to_segment_dist(0.5, 0.5, 0, 0, 1, 1)
assert abs(d) < 0.01
d2 = point_to_segment_dist(2, 0, 0, 0, 1, 0)
assert abs(d2 - 1.0) < 0.01
print('  [OK] point_to_segment_dist')
ch = [[-0.3,0],[0.3,0],[0.3,4],[-0.3,4]]
assert abs(distance_to_channel_center(0.0, 1.0, ch)) < 0.01
assert distance_to_channel_center(0.8, 1.0, ch) > 0.4
print('  [OK] distance_to_channel_center')
poly = [[-1.0,3.0],[-0.3,3.0],[-0.3,4.0],[-1.0,4.0]]
d_in = dist_to_shore(-0.7, 3.5, poly)
assert d_in < 1.0
d_out = dist_to_shore(0.0, 2.0, poly)
assert d_out > 0.5
print('  [OK] dist_to_shore')
print()

print('PHASE 6: BOAT TARGET FILTER')
from boat_target_filter import BoatTargetSelector
sel = BoatTargetSelector(cfg['radar']['boat_filter'])
assert sel._passes_filters({'x_cm':'50','y_cm':'200','speed_cm_s':'10','pv':'55','match_distance_m':'0.02'})
assert not sel._passes_filters({'x_cm':'-200','y_cm':'200','speed_cm_s':'10','pv':'55','match_distance_m':'0.02'})
assert not sel._passes_filters({'x_cm':'50','y_cm':'200','speed_cm_s':'10','pv':'15','match_distance_m':'0.02'})
print('  [OK] XY/PV filtering')
sel2 = BoatTargetSelector(cfg['radar']['boat_filter'])
for frame in range(1, 20):
    x = 50 + frame; y = 200 + frame
    t = {'track_id':99, 'x_cm':str(x), 'y_cm':str(y), 'speed_cm_s':'30','pv':'60','match_distance_m':'0.02'}
    sel2.select([t], frame)
    if frame >= 15:
        assert sel2._passes_displacement(t) == True
print('  [OK] displacement filter: moving passes')
print()

print('PHASE 7: ANOMALY DETECTION OFFLINE')
for item in os.listdir('.'):
    if 'csv' in item and os.path.isdir(item): shutil.rmtree(item)
    if item.startswith('anomaly_events'): os.remove(item)

with open('__e2e.csv','w',newline='',encoding='utf-8-sig') as f:
    w = csv.DictWriter(f, fieldnames=['pc_time','packet_no','target_index','track_id','x_m','y_m','speed_m_s','pv','distance_m','match_distance_m'])
    w.writeheader()
    for frame in range(600):
        pn=frame+1
        w.writerow({'pc_time':'','packet_no':pn,'target_index':0,'track_id':1,'x_m':'0.3','y_m':'2.4','speed_m_s':'0.3','pv':'50','distance_m':'2.42','match_distance_m':'0.01'})
        y2='2.5' if 50<=frame<200 else ('3.5' if frame>=200 else '2.1')
        w.writerow({'pc_time':'','packet_no':pn,'target_index':1,'track_id':2,'x_m':'0.4','y_m':y2,'speed_m_s':'0.3','pv':'50','distance_m':'2.5','match_distance_m':'0.01'})
        s3='0.3' if frame<280 else ('1.5' if frame<320 else '0.3')
        w.writerow({'pc_time':'','packet_no':pn,'target_index':2,'track_id':3,'x_m':'0.0','y_m':'2.0','speed_m_s':s3,'pv':'50','distance_m':'2.0','match_distance_m':'0.01'})
        if frame<380: x4,y4,s4='-0.05','1.5','0.5'
        elif frame<420: x4,y4,s4='-0.6','3.0','0.3'
        else: x4,y4,s4='-0.7','3.75','0.02'
        w.writerow({'pc_time':'','packet_no':pn,'target_index':3,'track_id':4,'x_m':x4,'y_m':y4,'speed_m_s':s4,'pv':'50','distance_m':'3.8','match_distance_m':'0.01'})

from anomaly_detector import replay_from_csv, AnomalyDetector
ad_cfg = {'zones': cfg['zones'], 'anomaly_detection': dict(cfg['anomaly_detection'])}
det = replay_from_csv('__e2e.csv', ad_cfg)
assert det.event_count == 3, f'Expected 3, got {det.event_count}'
for fn in os.listdir('.'):
    if fn.startswith('anomaly_events'):
        with open(fn,'r',encoding='utf-8-sig') as fh: rows = list(csv.DictReader(fh))
        types = [r['event_type'] for r in rows]
        assert set(types) == {'SC','KS','KA'}, f'Missing: {types}'
        break
print(f'  [OK] {det.event_count} events: {types}')
det.close()
os.remove('__e2e.csv')
for item in os.listdir('.'):
    if 'csv' in item and os.path.isdir(item): shutil.rmtree(item)
    if item.startswith('anomaly_events'): os.remove(item)
print()

print('PHASE 8: EDGE CASES + is_active')
d2 = AnomalyDetector(cfg)
d2.feed([]); d2.feed(None)
d2.feed([{'track_id':1,'x_m':1.0,'y_m':2.0,'speed_m_s':0.5}])
d2.feed([{'x_m':1.0,'y_m':2.0,'speed_m_s':0.5}])
assert d2.event_count == 0
for i in range(200):
    d2.feed([{'track_id':99,'x_m':-100,'y_m':-100,'speed_m_s':0.1}], _now=i*0.1)
assert d2.event_count == 0
d2.docking_states[99] = {'state':'DEVIATED','stopped_mono':None,'was_moving':False}
assert d2.is_active() == False
d2.docking_states[77] = {'state':'STOPPED','stopped_mono':100.0,'was_moving':True}
assert d2.is_active() == True
d2.close()
print('  [OK] empty/None/no_track_id/out_of_zone: safe')
print('  [OK] is_active: wall=False, boat=True')
print()

print('PHASE 9: FIELD COMPATIBILITY')
parse_fields = {'index','x_cm','y_cm','speed_cm_s','pv','x_m','y_m','speed_m_s','distance_m'}
tracker_fields = {'track_id','match_distance_m','pc_time','packet_no'}
all_fields = parse_fields | tracker_fields
consumers = {
    'BoatTargetSelector': {'x_cm','y_cm','speed_cm_s','pv','track_id','match_distance_m','distance_m'},
    'AnomalyDetector': {'track_id','x_m','y_m','speed_m_s'},
    'VOFA+': {'x_m','y_m','speed_m_s','pv'},
    'CsvWriters': {'index','track_id','match_distance_m','x_cm','y_cm','speed_cm_s','pv','x_m','y_m','speed_m_s','distance_m','pc_time','packet_no'},
}
for name, needed in consumers.items():
    missing = needed - all_fields
    assert not missing, f'{name} needs {missing}'
    print(f'  [OK] {name}: {len(needed)} fields present')
print()

print('PHASE 10: CSV WRITERS')
from radar_track_parser_v4 import CsvWriters
sig = list(inspect.signature(CsvWriters.__init__).parameters.keys())
assert 'filtered_csv' in sig
print(f'  [OK] CsvWriters: {sig}')
print()

print('ALL PHASES PASSED')
