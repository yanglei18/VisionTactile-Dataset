"""Display already-matched camera/pose pairs or strict groups, not raw samples.

Transport metadata and binary clouds arrive independently. Keep a previously
matched presentation briefly instead of drawing a half-delivered replacement.
All poses stay with their original clouds; source timestamps are untouched.
Independent live mode advances/leases each camera separately.
"""
from copy import deepcopy
import time


class CloudPresentation:
    HOLD_S = .5
    # Live preview may show a labelled, frozen pair during short input gaps.
    # This is not an admission window for new data or a recording policy.
    INDEPENDENT_HOLD_S = 3.

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.clear()

    def clear(self):
        self.frame = None
        self.clouds = {}
        self._signature = None
        self._key = None
        self._accepted = 0.
        self.held = False
        self.hold_age_s = 0.
        self.stale_cameras = set()
        self._camera_cache = {}
        self._independent_signature = None

    @staticmethod
    def _identity(frame):
        return (
            tuple(frame.get(k) for k in ('mode', 'manager_instance_id', 'scene_epoch',
                  'time_epoch', 'coordinate_epoch', 'sync_epoch', 'world_frame', 'time_mode')),
            frame.get('display_from_world'), frame.get('map_identity'),
            tuple((c.get('id'), c.get('geometry_revision'), c.get('geometry_sha256'),
                   c.get('registration_state')) for c in frame.get('cameras', ())),
        )

    def select(self, frame, clouds, *, enabled=True):
        if not enabled or not frame or frame.get('mode') != 'live':
            self.clear()
            return frame, clouds
        if frame.get('time_mode') == 'independent':
            return self._select_independent(frame,clouds)
        if self._independent_signature is not None:
            self.clear()
        now = self.clock()
        signature = self._identity(frame)
        if signature != self._signature:
            self.clear()
        self._signature = deepcopy(signature)
        # A source or transport stall must not extend the display indefinitely.
        age = now-self._accepted
        recent = bool(self.clouds) and 0 <= age < self.HOLD_S
        generated = frame.get('generated_realtime_ns')
        if recent and isinstance(generated, int):
            recent = all(0 <= generated-c['header']['source_realtime_ns'] < 1_000_000_000
                         for c in self.clouds.values())
        # Never merge old and new groups. Keep all old poses with all old clouds.
        if recent and not set(self.clouds).issubset(clouds):
            self.held = True
            self.hold_age_s = age
            held = deepcopy(self.frame)
            for kind in ('cameras', 'trackers'):
                for entity in held.get(kind, ()):
                    if isinstance(entity.get('age_s'), (float, int)):
                        entity['age_s'] += age
                        if entity['age_s'] > .25 and entity.get('state') == 'fresh':
                            entity['state'] = 'delayed'
            return held, dict(self.clouds)
        key = tuple(sorted((k, v['header']['cloud_sequence']) for k, v in clouds.items()))
        if key != self._key:
            self._accepted = now
            self._key = key
            self.frame = deepcopy(frame)
            self.clouds = dict(clouds)
        self.held = False
        self.hold_age_s = 0.
        return frame, clouds

    def _select_independent(self, frame, clouds):
        # Retain whole camera/Tracker/cloud pairs, not whole multi-camera groups.
        # Exact source poses travel with the held cloud. Never put an old cloud
        # on a new live Tracker pose or extend its lease on every GUI refresh.
        signature=self._identity(frame)[:3]
        if signature!=self._independent_signature:
            self.clear()
        self._independent_signature=deepcopy(signature)
        now=self.clock(); generated=frame.get('generated_realtime_ns')
        shown=deepcopy(frame); visible={}; trackers={t['role']:t for t in shown.get('trackers',())}
        paired_trackers={}; self.held=False; self.hold_age_s=0.; self.stale_cameras=set()
        names={c['id'] for c in shown.get('cameras',())}
        self._camera_cache={k:v for k,v in self._camera_cache.items() if k in names}
        for index,camera in enumerate(shown.get('cameras',())):
            name=camera['id']; role=camera.get('tracker_role'); tracker=trackers.get(role)
            identity=tuple(camera.get(k) for k in ('geometry_revision','geometry_sha256','registration_state','tracker_role'))+(tracker.get('tracker_id') if tracker else None,)
            old=self._camera_cache.get(name)
            if old and old['identity']!=identity:
                self._camera_cache.pop(name); old=None
            cloud=clouds.get(name)
            if camera.get('registration_state')!='ready':
                self._camera_cache.pop(name,None); continue
            if cloud is not None:
                header=cloud['header']; key=(header.get('worker_instance_id'),header['cloud_sequence'])
                bound=header.get('tracker_pose',{})
                binding_ok=(header.get('matching_mode')=='independent' and tracker is not None
                            and bound.get('role')==role and bound.get('tracker_id')==tracker.get('tracker_id'))
                fresh_source=isinstance(generated,int) and 0<=generated-header['source_realtime_ns']<1_000_000_000
                if binding_ok and fresh_source and (old is None or old['key']!=key):
                    paired=deepcopy(tracker) if tracker else None
                    if paired and bound:
                        transform=bound['world_from_tracker']
                        paired.update(position_m=deepcopy(transform['translation_m']),quaternion_xyzw=deepcopy(transform['quaternion_xyzw']),
                                      source_realtime_ns=header['source_realtime_ns'],valid=True,reason='',
                                      before_ns=bound['before_ns'],after_ns=bound['after_ns'],gap_ns=bound['gap_ns'])
                    source_camera=deepcopy(camera); camera_pose=header['world_from_color']
                    source_camera.update(frame_key=deepcopy(header['frame_key']),
                        source_realtime_ns=header['source_realtime_ns'],
                        position_m=deepcopy(camera_pose['translation_m']),
                        quaternion_xyzw=deepcopy(camera_pose['quaternion_xyzw']))
                    old=dict(identity=identity,key=key,accepted=now,camera=source_camera,tracker=paired,cloud=cloud,
                             source_age_at_accept=(generated-header['source_realtime_ns'])/1e9)
                    self._camera_cache[name]=old
            if old is None: continue
            age=now-old['accepted']; source=old['cloud']['header']['source_realtime_ns']
            source_age=(generated-source)/1e9 if isinstance(generated,int) else -1.
            # A stalled scene timestamp must not freeze the displayed age.
            source_age=max(source_age,old['source_age_at_accept']+age)
            if not (0<=age<self.INDEPENDENT_HOLD_S and 0<=source_age<self.INDEPENDENT_HOLD_S):
                # Keep the sequence fence until a genuinely new packet arrives.
                continue
            held=(cloud is None or old['key']!=(cloud['header'].get('worker_instance_id'),cloud['header']['cloud_sequence'])
                  or age>=self.HOLD_S or source_age>=1.)
            self.held|=held
            if held: self.hold_age_s=max(self.hold_age_s,age)
            presented=deepcopy(old['camera']); presented['age_s']=source_age
            presented['state']='stale' if source_age>=1. else 'fresh' if source_age<=.25 else 'delayed'
            if source_age>=1.: self.stale_cameras.add(name)
            shown['cameras'][index]=presented; visible[name]=old['cloud']
            paired=old['tracker']
            if paired and (role not in paired_trackers or source>paired_trackers[role]['source_realtime_ns']):
                paired=deepcopy(paired); paired.update(age_s=source_age,state=presented['state'])
                paired_trackers[role]=paired
        shown['trackers']=[paired_trackers.get(t['role'],t) for t in shown.get('trackers',())]
        self.frame=shown; self.clouds=visible
        return shown,visible
