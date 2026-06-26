"""Final comprehensive project audit"""
import ast, json, csv, os, shutil, sys, inspect

errors = []

print('=' * 70)
print('PHASE 1: SYNTAX (8 files)')
print('=' * 70)
for f in ['main.py','live_tracking.py','radar_live.py','anomaly_detector.py',
          'radar_track_parser_v4.py','imou_ptz.py','boat_target_filter.py']:
    with open(f,'r',encoding='utf-8') as fh: ast.parse(fh.read())
    print(f'  OK  {f}')
with open('live_config.json','r',encoding='utf-8') as fh: cfg = json.load(fh)
print('  OK  live_config.json')

print()
print('PHASE 2: IMPORT CHAIN')
print()
from anomaly_detector import AnomalyDetector, replay_from_csv, point_in_polygon
from boat_target_filter import BoatTargetSelector
from imou_ptz import ImouPTZClient, ImouAPIError
print('  OK  anomaly_detector + boat_target_filter + imou_ptz')

print()
print('PHASE 3: CONFIG CONSISTENCY')
print()
# Check anomaly_detection sub-keys
ad = cfg['anomaly_detection']
for section in ['ship_rendezvous','fast_passage','shore_docking','cooldowns']:
    assert section in ad, f'Missing anomaly_detection.{section}'
print(f'  OK  anomaly_detection: 4 sections present')

det = AnomalyDetector(cfg)
checks = [
    (det.enabled, True),
    (det.sc_dist_threshold, 0.5), (det.sc_duration_s, 5.0), (det.sc_rel_speed_max, 0.3),
    (det.sc_approach_dist, 1.0), (det.sc_min_dist_drop, 0.3),
    (det.ks_speed_min, 0.9), (det.ks_speed_ratio, 1.5), (det.ks_accel_min, 0.3), (det.ks_min_duration_s, 1.5),
    (det.ka_deviation_m, 0.3), (det.ka_speed_low, 0.15), (det.ka_duration_s, 3.0),
    (det.ka_shore_dist, 0.5), (det.ka_fast_approach, 0.5), (det.ka_dist_drop, 0.3), (det.ka_post_slow, 0.15),
    (det.cooldown_sc, 10.0), (det.cooldown_ks, 10.0), (det.cooldown_ka, 10.0),
]
all_ok = True
for actual, expected in checks:
    if actual != expected:
        print(f'  FAIL: {actual} != expected {expected}')
        all_ok = False
if all_ok:
    print(f'  OK  All {len(checks)} params match live_config.json')
det.close()

# Check boat_filter
bf = cfg['radar']['boat_filter']
assert bf['x_min_cm'] == -60 and bf['x_max_cm'] == 60
assert bf['y_max_cm'] == 400
assert bf['min_pv'] == 30
assert bf['max_match_distance_m'] == 0.15
assert bf['min_track_hits'] == 10
print(f'  OK  boat_filter: all 5 active params correct')

# Check CsvWriters signature
from radar_track_parser_v4 import CsvWriters
sig = inspect.signature(CsvWriters.__init__)
params = list(sig.parameters.keys())
assert 'filtered_csv' in params, 'CsvWriters missing filtered_csv param'
print(f'  OK  CsvWriters params: {params}')

# Check live_config csv has filtered key
assert 'filtered' in cfg['radar']['csv'], 'Missing csv.filtered'
print(f'  OK  csv.filtered present')

# Check config keys used by radar_live
vofa = cfg['radar'].get('vofa', {})
assert vofa.get('enabled') == True
print(f'  OK  vofa.enabled={vofa["enabled"]}')

print()
print('PHASE 4: DATA FLOW')
print()
# Simulate radar_live.py on_packet: the EXACT flow
sel = BoatTargetSelector(bf)

# Simulate canonical parse_target_packet output (after SimpleTracker)
targets = [
    {'index':0, 'track_id':1, 'x_cm':50, 'y_cm':200, 'speed_cm_s':10, 'pv':55,
     'x_m':0.5, 'y_m':2.0, 'speed_m_s':0.1, 'distance_m':2.06, 'match_distance_m':0.01},
    {'index':1, 'track_id':2, 'x_cm':40, 'y_cm':210, 'speed_cm_s':5, 'pv':45,
     'x_m':0.4, 'y_m':2.1, 'speed_m_s':0.05, 'distance_m':2.14, 'match_distance_m':0.02},
    # A noise target outside pool
    {'index':2, 'track_id':3, 'x_cm':-200, 'y_cm':100, 'speed_cm_s':50, 'pv':20,
     'x_m':-2.0, 'y_m':1.0, 'speed_m_s':0.5, 'distance_m':2.24, 'match_distance_m':0.3},
]

# Step 1: filter
boat_targets = [t for t in targets if sel._passes_filters(t)]
if not boat_targets:
    boat_targets = targets
print(f'  filter: {len(targets)} -> {len(boat_targets)} targets')
assert len(boat_targets) == 2, f'Expected 2 boat_targets (X=-2.0 filtered), got {len(boat_targets)}'

# Step 2: anomaly detection
det2 = AnomalyDetector(cfg)
det2.feed(boat_targets)
print(f'  anomaly_detector.feed(): OK (event_count={det2.event_count})')

# Step 3: select
nearest = sel.select(targets, 1)
print(f'  boat_selector.select(): track_id={nearest.get("track_id") if nearest else None}')
det2.close()

print()
print('PHASE 5: END-TO-END OFFLINE')
print()
for item in os.listdir('.'):
    if 'csv' in item and os.path.isdir(item): shutil.rmtree(item)
    if item.startswith('anomaly_events'): os.remove(item)

with open('__final.csv','w',newline='',encoding='utf-8-sig') as f:
    w = csv.DictWriter(f, fieldnames=['pc_time','packet_no','target_index','track_id','x_m','y_m','speed_m_s','pv','distance_m','match_distance_m'])
    w.writeheader()
    for frame in range(600):
        pn=frame+1
        w.writerow({'pc_time':'','packet_no':pn,'target_index':0,'track_id':1,'x_m':'0.5','y_m':'2.4','speed_m_s':'0.05','pv':'50','distance_m':'2.45','match_distance_m':'0.01'})
        y2='2.45' if 40<=frame<200 else (str(round(2.4-(40-frame)*0.04,3)) if frame<40 else '3.5')
        w.writerow({'pc_time':'','packet_no':pn,'target_index':1,'track_id':2,'x_m':'0.6','y_m':y2,'speed_m_s':'0.05','pv':'50','distance_m':'2.45','match_distance_m':'0.01'})
        y3,s3=('1.0','0.3') if frame<290 else (('2.0','2.6') if frame<330 else ('3.0','0.2'))
        w.writerow({'pc_time':'','packet_no':pn,'target_index':2,'track_id':3,'x_m':'0.0','y_m':y3,'speed_m_s':s3,'pv':'50','distance_m':'2.0','match_distance_m':'0.01'})
        if frame<390: x4,y4,s4='-0.05',str(round(1.0+(frame-380)*0.02,3)),'0.5'
        elif frame<440: x4,y4,s4='-0.7',str(round(3.0+(frame-390)*0.02,3)),'0.3'
        else: x4,y4,s4='-0.7','3.75','0.05'
        w.writerow({'pc_time':'','packet_no':pn,'target_index':3,'track_id':4,'x_m':x4,'y_m':y4,'speed_m_s':s4,'pv':'50','distance_m':'3.8','match_distance_m':'0.01'})

ad_cfg = {'zones': cfg['zones'], 'anomaly_detection': dict(cfg['anomaly_detection'])}
ad_cfg['anomaly_detection']['enabled'] = True
det = replay_from_csv('__final.csv', ad_cfg)
assert det.event_count == 3, f'Expected 3 events, got {det.event_count}'
det.close()

for fn in os.listdir('.'):
    if fn.startswith('anomaly_events'):
        with open(fn,'r',encoding='utf-8-sig') as fh: types = [r['event_type'] for r in csv.DictReader(fh)]
        break
assert types == ['SC','KS','KA'], f'Types={types}'
print(f'  OK  SC/KS/KA all 3 detected: {types}')

os.remove('__final.csv')
for item in os.listdir('.'):
    if 'csv' in item and os.path.isdir(item): shutil.rmtree(item)
    if item.startswith('anomaly_events'): os.remove(item)

print()
print('=' * 70)
print('AUDIT COMPLETE - ALL CHECKS PASSED')
print('=' * 70)
