"""Explicit touch capture and shape-only transactions for the live controller.

The user supplies contact labels; neither IMUs nor rendered meshes establish
contact. Optimization is isolated from acquisition. Preview never changes the
measurement pose, its exported geometry, or an installation/functional matrix.
"""
from copy import deepcopy
import json
from pathlib import Path
from uuid import uuid4

import numpy as np

from .config import digest

SHAPE_COMMANDS = {
    'capture_shape_contact': {'category', 'contact_confirmed'},
    'optimize_shape': set(), 'cancel_shape': set(), 'preview_shape': {'mode'},
    'apply_shape_candidate': set(), 'load_shape_profile': {'path'},
    'save_shape_profile': {'path'}, 'revert_shape': set(),
    'save_shape_task': {'path'}, 'load_shape_task': {'path'},
    'retry_calibration_save': set(), 'retry_calibration_save_to': {'directory'},
}
CONTACT_NAMES = {2: '食指', 3: '中指', 4: '无名指', 5: '小指'}
CONTACT_SLOTS = {2: (1, 2), 3: (4, 5), 4: (10, 11), 5: (7, 8)}


def valid_shape_command(name, args):
    if name not in SHAPE_COMMANDS or set(args) != SHAPE_COMMANDS[name]:
        return False
    if name == 'capture_shape_contact':
        return type(args['category']) is int and args['category'] in CONTACT_NAMES and args['contact_confirmed'] is True
    if name == 'preview_shape':
        return args['mode'] in ('current', 'candidate', 'baseline')
    if 'path' in args:
        return isinstance(args['path'], str) and bool(args['path'].strip()) and len(args['path']) <= 4096
    if 'directory' in args:
        return isinstance(args['directory'], str) and bool(args['directory'].strip()) and len(args['directory']) <= 4096
    return True


class ShapeCalibrationMixin:
    def _init_shape(self, job_factory=None):
        self._shape_job_factory = job_factory
        self._shape_job = self._shape_job_context = None
        self._shape_capture = None
        self._shape_contacts = {}
        self._shape_candidate = None
        self._shape_applied = None
        self._shape_applied_path = None
        self._shape_previous = None
        self._shape_token = self._shape_preview_revision = 0
        self._shape_preview = 'current'
        self._shape_preview_cache = None
        self._shape_status, self._shape_message, self._shape_error = 'waiting', '完成基础标定后，依次采集四组真实触指姿态。', None
        self._shape_command_generation = 0
        self._shape_model_identity = None
        self._shape_reset_task()

    def _shape_reset_task(self):
        self._shape_task_id = uuid4().hex
        self._shape_task_path = self._shape_task_directory = None
        self._shape_task_basis = None
        self._shape_task_loaded = False
        self._shape_saved_groups = set()
        self._shape_storage_error = None

    def _shape_connected(self):
        return bool(self._session and self._session.opened and not self._session.stop.is_set()
                    and not self._session.done.is_set())

    def _shape_context(self):
        # Imported evidence stays bound to its original session, never to an
        # unrelated live profile. Cancellation still invalidates both job types.
        return ('saved_task', self._shape_task_id) if self._shape_task_loaded else self._shape_binding()

    def _shape_task_document(self):
        if not self._shape_contacts:
            raise ValueError('尚无已完成的触指数据。')
        if self._shape_task_basis is None:
            provenance = self._shape_expected_provenance()
            self._shape_task_basis = dict(schema_version=1, kind='fsglove_touch_task',
                task_id=self._shape_task_id, binding=list(self._shape_binding()),
                source_session_id=self._session.session_id,
                upstream={k: v for k, v in provenance.items() if k != 'mano_model'},
                mano_model=provenance['mano_model'], contact_labels_confirmed=True,
                contact_labels_source='user_explicit_not_inferred_from_imu',
                before_shape_params=list(self._profile.get('shape_params', [0.] * 10)))
        return {**deepcopy(self._shape_task_basis),
                'groups': {str(k): dict(binding=list(v['binding']), rows=deepcopy(v['rows']))
                           for k, v in self._shape_contacts.items()}}

    def _shape_checkpoint(self, path=None):
        """Small immutable task revisions, independent of the bulk-log latch.

        Failed writes leave every captured row in memory. Only an explicit retry
        or another completed group writes again; pose ticks never retry I/O.
        """
        from .calibration_artifacts import save_task
        try:
            explicit = path is not None
            if path is None:
                directory = (Path(self._shape_task_directory) if self._shape_task_directory
                             else self._session.directory / 'calibration')
                path = directory / f'touch_{self._shape_task_id}.{uuid4().hex[:12]}.json'
            resolved = save_task(path, self._shape_task_document(), self._shape_expected_provenance())
        except (OSError, ValueError, TypeError) as exc:
            self._shape_storage_error = str(exc)
            return False
        self._shape_task_path = resolved
        if explicit:
            self._shape_task_directory = str(Path(resolved).parent)
        self._shape_saved_groups = set(self._shape_contacts)
        self._shape_storage_error = None
        return True

    def _shape_busy(self):
        return self._shape_capture is not None or self._shape_job is not None

    def _shape_binding(self):
        p = self._profile or {}
        return (getattr(self._session, 'session_id', None), self._clear_generation,
                self._config_sha, p.get('upstream_state_sha256'),
                digest(p.get('nodes', {})), digest(p.get('functional_fingers', {})),
                p.get('functional_enabled', True))

    def _shape_state(self, now):
        capture = self._shape_capture
        preparing = capture is not None and now < capture['collect_at']
        connected = self._shape_connected()
        ready = connected and self._profile is not None and self._active is None and self._pose_backend is not None
        idle = not self._shape_busy() and self._active is None
        offline_ready = self._shape_task_loaded and not connected and self._pose_backend is not None
        storage = self._calibration_storage_status(self._calibration_retry_session())
        unsaved = set(self._shape_contacts) - self._shape_saved_groups
        candidate = self._shape_candidate
        return dict(status=self._shape_status, message=self._shape_message, error=self._shape_error,
            busy=self._shape_busy(), active_category=capture['category'] if capture else None,
            phase=('preparing' if preparing else 'collecting') if capture else None,
            remaining_s=max(0., (capture['collect_at'] if preparing else capture['end_at']) - now) if capture else 0.,
            contacts={str(k): dict(frames=len(v['rows']), confirmed=True, saved=k in self._shape_saved_groups)
                      for k, v in self._shape_contacts.items()},
            task_path=self._shape_task_path, task_loaded=self._shape_task_loaded,
            can_save_task=idle and bool(self._shape_contacts), can_load_task=idle and not connected,
            saved_groups=[str(k) for k in sorted(self._shape_saved_groups)],
            unsaved_groups=[str(k) for k in sorted(unsaved)],
            can_retry_save=idle and bool(unsaved or self._shape_storage_error or storage['retry_available']),
            storage_error=self._shape_storage_error,
            storage_message=('触指数据已保留在内存，但文件保存未完成；可重试或另选保存目录。'
                             if self._shape_storage_error else '已完成的接触组已保存。' if self._shape_task_path else None),
            candidate_available=candidate is not None,
            candidate_path=candidate.get('shape_path') if candidate else None,
            metrics=deepcopy(candidate.get('metrics', {})) if candidate else {},
            shape_calibrated=bool((self._profile or {}).get('shape_calibrated', self._shape_applied is not None)),
            shape_source='personalized' if self._shape_applied else 'configured' if (self._profile or {}).get('shape_calibrated') else 'default',
            applied_path=self._shape_applied_path, can_revert=self._shape_previous is not None,
            preview_mode=self._shape_preview, preview_revision=self._shape_preview_revision,
            can_capture=ready and not self._shape_busy() and not self._shape_task_loaded,
            can_optimize=(ready or offline_ready) and idle and len(self._shape_contacts) == 4,
            can_apply=ready and not self._shape_busy() and candidate is not None
                and candidate.get('shape_profile') != self._shape_applied,
            can_export=candidate is not None or self._shape_applied is not None)

    def _shape_publish(self):
        with self._state_lock:
            self._snapshot['calibration'] = self._calibration_state(self._clock())
            self._revision += 1

    def _shape_invalidate(self, message='标定参考已改变；触指数据需重新采集。'):
        self._shape_token += 1
        if self._shape_job is not None:
            self._shape_job.cancel()
        self._shape_capture = None
        self._shape_contacts = {}
        self._shape_reset_task()
        self._shape_candidate = None
        self._shape_preview = 'current'
        self._shape_preview_revision += 1
        self._shape_preview_cache = None
        self._shape_status = 'applied' if self._shape_applied else 'waiting'
        self._shape_message, self._shape_error = message, None

    def _shape_expected_provenance(self):
        core = getattr(self._pose_backend, 'core', None)
        if core is None:
            raise ValueError('当前后端不支持官方手形标定。')
        provenance = deepcopy(core.provenance)
        if self._shape_model_identity is None:
            from .shape_calibration import model_provenance
            self._shape_model_identity = model_provenance(core.mano_assets_root, hand=self.config["hand"])
        provenance['mano_model'] = self._shape_model_identity
        return provenance

    def _shape_ready(self):
        session = self._session
        if not session or not session.opened or session.stop.is_set() or session.done.is_set() or self._profile is None:
            raise ValueError('请先连接并完成本次三姿态基础标定。')
        if self._active is not None or self._shape_busy():
            raise ValueError('已有采集或优化正在进行；请等待完成，或取消本次手形操作。')
        return session

    def _shape_command(self, name, args):
        try:
            self._shape_error = None
            self._shape_execute(name, args)
        except (ValueError, OSError, RuntimeError, KeyError, TypeError) as exc:
            # An optimization/import/export failure must not disconnect a live
            # sensor or discard a usable installed profile.
            self._shape_error = str(exc)
            self._shape_message = '本次手形操作未完成；原标定和实时采集保持不变。'
            if not self._shape_busy():
                self._shape_status = 'failed'
        finally:
            self._shape_command_generation += 1
            with self._state_lock:
                self._snapshot['runtime'].update(shape_command_generation=self._shape_command_generation,
                                                shape_command_error=self._shape_error)
            self._shape_publish()

    def _shape_execute(self, name, args):
        from .shape_calibration import validate_shape_profile
        from .calibration_artifacts import load_task, preflight, task_to_contacts, write_artifact
        if name == 'cancel_shape':
            self._shape_token += 1
            if self._shape_job is not None:
                self._shape_job.cancel()
            self._shape_capture = None
            self._shape_status, self._shape_message = 'cancelled', '本次采集/优化已取消；已完成接触组和原手形保留。'
            return
        if name in ('save_shape_task', 'load_shape_task', 'retry_calibration_save', 'retry_calibration_save_to'):
            if self._shape_busy() or self._active is not None:
                raise ValueError('请先结束当前采集/优化，再保存或恢复标定任务。')
            if name == 'load_shape_task':
                if self._shape_connected():
                    raise ValueError('请在未连接传感器时恢复离线触指任务；不会覆盖当前实时标定。')
                path = Path(args['path']).expanduser().resolve()
                task = load_task(path, self._shape_expected_provenance())
                # Fully validate before touching any current evidence/state.
                self._shape_invalidate()
                self._shape_task_id, self._shape_task_path = task['task_id'], str(path)
                self._shape_task_directory = str(path.parent)
                self._shape_task_basis = {k: deepcopy(v) for k, v in task.items() if k != 'groups'}
                self._shape_contacts = {int(k): {**v, 'binding': tuple(v['binding'])}
                                        for k, v in task['groups'].items()}
                self._shape_task_loaded = True
                self._shape_saved_groups = set(self._shape_contacts)
                self._shape_status = 'ready' if len(self._shape_contacts) == 4 else 'waiting'
                self._shape_message = (f'已恢复 {len(self._shape_contacts)}/4 组已保存触指数据；'
                    '四组齐全可直接离线优化，不会连接传感器或恢复旧安装参数。')
                return
            if name == 'save_shape_task':
                if not self._shape_checkpoint(args['path']):
                    raise OSError(self._shape_storage_error)
                self._shape_message = f'触指任务已另存为 {self._shape_task_path}。'
                return
            target = args.get('directory')
            if target is not None:
                # Check before retargeting future writes; failed probes retain
                # the previous destination as well as all current evidence.
                self._shape_task_directory = preflight(target)
            ok = True
            owner = self._calibration_retry_session()
            if owner is not None:
                ok = self._retry_calibration_saves(owner, directory=target)
            if self._shape_contacts and (self._shape_storage_error or
                    set(self._shape_contacts) != self._shape_saved_groups):
                ok = self._shape_checkpoint() and ok
            self._shape_message = ('标定文件已保存；调试日志限额和实时采集状态不变。' if ok else
                                   '仍有标定文件未保存，数据保留在内存；请另选保存目录后重试。')
            return
        if name == 'save_shape_profile':
            if self._shape_busy():
                raise ValueError('请等当前采集/优化完成后导出。')
            document = self._shape_candidate['shape_profile'] if self._shape_candidate else self._shape_applied
            if document is None:
                raise ValueError('尚无可导出的个性化手形。')
            path = write_artifact(args['path'], document)
            self._shape_message = f'手形已另存为 {path}；未改变当前应用状态。'
            return
        if name == 'load_shape_profile':
            if self._shape_busy() or self._active is not None:
                raise ValueError('请先结束当前采集/优化，再载入手形。')
            path = Path(args['path']).expanduser().resolve()
            if path.stat().st_size > 4 * 1024 * 1024:
                raise ValueError('手形文件过大；请选择优化器导出的 JSON。')
            with path.open(encoding='utf-8') as stream:
                document = validate_shape_profile(json.load(stream), self._shape_expected_provenance())
            self._shape_commit(document, str(path), event='shape_loaded')
            return
        if name == 'preview_shape':
            self._shape_ready()
            if args['mode'] in ('candidate', 'baseline') and self._shape_candidate is None:
                raise ValueError('请先完成优化，才能比较候选和优化前手形。')
            self._shape_preview = args['mode']
            self._shape_preview_revision += 1
            self._shape_preview_cache = None
            self._shape_message = '预览只改变显示；ROS 和记录仍使用已应用手形。'
            return
        if name == 'revert_shape':
            if self._shape_busy() or self._active is not None:
                raise ValueError('请先结束当前采集/优化。')
            if self._shape_previous is None:
                raise ValueError('没有可恢复的上一手形。')
            previous = self._shape_previous
            self._shape_previous = None
            self._shape_applied, self._shape_applied_path = previous['document'], previous['path']
            if self._profile is not None and self._session is not None:
                p = {**deepcopy(self._profile), **deepcopy(previous['fields'])}
                self._save_profile(self._session, p, event='shape_reverted')
            self._shape_preview = 'current'
            self._shape_preview_revision += 1
            self._shape_preview_cache = None
            self._shape_status = 'applied' if self._shape_applied else 'ready'
            self._shape_message = '已恢复上一手形；安装参数与五指功能轴保持不变。'
            return
        if name == 'optimize_shape':
            if self._shape_task_loaded:
                if self._shape_connected() or self._active is not None or self._shape_busy():
                    raise ValueError('恢复的触指任务仅支持在未连接、无其他采集/优化时离线优化。')
                session = None
            else:
                session = self._shape_ready()
                if any(c['binding'] != self._shape_binding() for c in self._shape_contacts.values()):
                    raise ValueError('接触数据不属于当前标定，请重新采集四组。')
            if len(self._shape_contacts) != 4:
                raise ValueError('请先确认并采齐拇指与其余四指的真实接触。')
            contacts = task_to_contacts(self._shape_task_document(), self._shape_expected_provenance())
            if set(self._shape_contacts) != self._shape_saved_groups and not self._shape_checkpoint():
                raise OSError(self._shape_storage_error)
            parent = Path(self._shape_task_directory) if self._shape_task_directory else session.directory
            try:
                preflight(parent)
            except OSError as exc:
                self._shape_storage_error = str(exc)
                raise
            from .shape_job import ShapeJob
            options = self._pose_backend.shape_job_options()
            options.update(iterations=500, seed=0)
            self._shape_job_context = (self._shape_token, self._shape_context())
            self._shape_job = (self._shape_job_factory or ShapeJob)(contacts, options, parent / ('shape_' + uuid4().hex[:12]))
            self._shape_status, self._shape_message = 'optimizing', '官方手形优化正在独立进程运行；实时采集与 ROS 发布不受影响。'
            if session is not None:
                self._event(session, 'shape_optimization_started', contact_frames=len(contacts['finger_category']))
            return
        session = self._shape_ready()
        if name == 'capture_shape_contact':
            if self._shape_task_loaded:
                raise ValueError('恢复的触指任务不能追加当前安装的数据；请使用新的标定会话采集。')
            with session.lock:
                frame = getattr(session.fusion, 'input_snapshot', session.fusion.snapshot)(self._clock())
            if self._capture_health(frame):
                raise ValueError('当前输入未恢复健康，暂不采集触指；原手形保留。')
            now = self._clock()
            self._shape_token += 1
            self._shape_candidate = None
            self._shape_preview = 'current'
            self._shape_preview_revision += 1
            self._shape_capture = dict(category=args['category'], collect_at=now + 3., end_at=now + 5.,
                rows=[], last_identity=None, last_bin=None, binding=self._shape_binding())
            self._shape_status = 'capturing'
            self._shape_message = f'拇指指尖轻触{CONTACT_NAMES[args["category"]]}指尖；准备 3 秒后保持接触 2 秒，不要用力挤压。'
            self._event(session, 'shape_contact_started', category=args['category'], contact_confirmed=True)
            return
        if name == 'apply_shape_candidate':
            if not self._shape_candidate:
                raise ValueError('没有可应用的候选手形。')
            document = validate_shape_profile(self._shape_candidate['shape_profile'], self._shape_expected_provenance())
            self._shape_commit(document, self._shape_candidate.get('shape_path'), event='shape_applied')

    def _shape_tick(self, session, frame, pose, now):
        capture = self._shape_capture
        if capture is None or not capture['collect_at'] <= now < capture['end_at']:
            return
        if capture['binding'] != self._shape_binding():
            self._shape_invalidate()
            return
        required_slots = (0, 14, 15, *CONTACT_SLOTS[capture['category']])
        required = {node for node, slot in self.config['node_to_slot'].items() if slot in required_slots}
        nodes = frame.get('nodes', {})
        if self._capture_health(frame) or any(node not in nodes for node in required):
            return
        if not pose.get('pose_available') or not pose.get('solver_success') or pose.get('calibration_rejections'):
            return
        if pose.get('display_only') or pose.get('held_slots') or pose.get('source_fresh') is False:
            return  # Held/render-only geometry and repeated solves are not new contact measurements.
        if set(required_slots) & set(pose.get('virtual_input_slots', ())):
            return
        identity = tuple((node, nodes[node].get('epoch'), nodes[node].get('sample_index')) for node in sorted(required))
        if identity == capture['last_identity'] or any(type(item[2]) is not int or item[2] <= 0 for item in identity):
            return
        if any(nodes[n].get('orientation_valid') is False
               or nodes[n].get('age_s', float('inf')) > self.config['stale_after_s']
               or nodes[n].get('input_filter', {}).get('gyro_rejected')
               or nodes[n].get('input_filter', {}).get('accel_rejected') for n in required):
            return
        value = np.asarray(pose.get('mano_pose_rotvec'), dtype=float)
        if value.shape != (16, 3) or not np.isfinite(value).all():
            return
        bucket = min(29, int((now - capture['collect_at']) * 15.))
        row = dict(category=capture['category'], pose=value.tolist(), timestamp_s=now,
                   input_identity=[list(x) for x in identity], contact_confirmed=True)
        if bucket == capture['last_bin']:
            capture['rows'][-1] = row
        else:
            capture['rows'].append(row)
        capture['last_bin'], capture['last_identity'] = bucket, identity

    def _poll_shape(self):
        capture = self._shape_capture
        if capture is not None and self._clock() >= capture['end_at']:
            self._shape_capture = None
            rows = capture['rows']
            good = (capture['binding'] == self._shape_binding() and len(rows) >= 10
                    and rows[-1]['timestamp_s'] - rows[0]['timestamp_s'] >= 1.
                    and capture['end_at'] - rows[-1]['timestamp_s'] <= self.config['stale_after_s'])
            if good:
                self._shape_contacts[capture['category']] = capture
                self._shape_saved_groups.discard(capture['category'])
                self._shape_status = 'ready' if len(self._shape_contacts) == 4 else 'waiting'
                self._shape_message = f'已采集拇指与{CONTACT_NAMES[capture["category"]]}接触，共 {len(rows)} 帧；已完成 {len(self._shape_contacts)}/4 组。'
                self._shape_message += ('已保存。' if self._shape_checkpoint() else
                    '文件保存失败，接触数据仍保留在内存；可重试或另存。')
            else:
                self._shape_status, self._shape_error = 'failed', '有效新鲜接触姿态不足；只需重采这一组，已有组保留。'
                self._shape_message = self._shape_error
            if self._session is not None:
                self._event(self._session, 'shape_contact_finished', category=capture['category'], frames=len(rows), accepted=good)
            self._shape_publish()
        job = self._shape_job
        if job is None or job.is_alive():
            return
        result, context = job.result or {}, self._shape_job_context
        job.join(0.)
        self._shape_job = self._shape_job_context = None
        if context == (self._shape_token, self._shape_context()) and not self._stop.is_set():
            report = result.get('report') or {}
            if result.get('error') or not report.get('candidate_available'):
                self._shape_status = 'failed' if report.get('status') != 'cancelled' else 'cancelled'
                self._shape_error = result.get('error') or report.get('error') or '未获得可用候选，原手形保留。'
                self._shape_message = self._shape_error
            else:
                self._shape_candidate = deepcopy(report)
                self._shape_status = 'candidate'
                self._shape_message = ('离线候选手形已保存；可导出个性化手形文件供下次加载，连接后仍需本次安装标定。'
                    if self._shape_task_loaded else
                    '候选手形已保存。请比较预览，再明确选择应用；尚未改变 ROS 和当前手形。')
                self._shape_error = None
                self._shape_preview_revision += 1
                if self._session is not None:
                    self._event(self._session, 'shape_candidate_ready', shape_path=report.get('shape_path'), metrics=report.get('metrics'))
        self._shape_publish()

    def _shape_commit(self, document, path, *, event):
        if document == self._shape_applied:
            # A repeated click or loading the same file must not replace the
            # genuine pre-application rollback with the applied shape itself.
            self._shape_message = '此手形已应用；保留上一手形恢复点。'
            return
        previous_profile = self._profile or (dict(shape_params=self._shape_applied['shape_params'],
            shape_calibrated=True, shape_provenance=self._shape_applied) if self._shape_applied else {})
        fields = {k:deepcopy(previous_profile.get(k, default)) for k, default in
                  (('shape_params', [0.] * 10), ('shape_calibrated', False), ('shape_provenance', None))}
        if self._profile is not None:
            fields['warnings'] = deepcopy(self._profile.get('warnings', []))
        self._shape_previous = dict(document=deepcopy(self._shape_applied), path=self._shape_applied_path, fields=fields)
        self._shape_applied, self._shape_applied_path = deepcopy(document), path
        if self._profile is not None and self._session is not None:
            profile = self._shape_with_applied(self._profile)
            self._save_profile(self._session, profile, event=event)
        self._shape_preview = 'current'
        self._shape_preview_revision += 1
        self._shape_preview_cache = None
        self._shape_status = 'applied'
        self._shape_message = ('个性化手形已应用；安装参数、五指功能轴与输入滤波器未重置。' if self._profile else
                               '手形已载入；连接后仍需本次安装标定，不会自动连接或复用旧安装参数。')

    def _shape_with_applied(self, profile):
        if self._shape_applied is None:
            return profile
        document = self._shape_applied
        profile = deepcopy(profile)
        profile.update(shape_params=list(document['shape_params']), shape_calibrated=True,
            shape_provenance=dict(algorithm=document['algorithm'], upstream=document.get('upstream'),
                mano_model=document.get('mano_model'), source_sha256=document.get('source_sha256'),
                profile_path=self._shape_applied_path, physical_accuracy_verified=False))
        profile['warnings'] = [v for v in profile.get('warnings', []) if '未声明完成手形触指优化' not in v]
        notice = '已应用个性化触指手形；接触拟合不代表人体精度通过，也不校正持续航向漂移。'
        if notice not in profile['warnings']:
            profile['warnings'].append(notice)
        return profile

    def _shape_render_preview(self, display_pose, display_geometry, pose, now):
        source = display_pose if display_pose is not None else pose
        if self._shape_preview == 'current' or self._shape_candidate is None:
            if display_pose is not None:
                display_pose = {**display_pose, 'shape_preview_mode': 'current', 'shape_preview_revision': self._shape_preview_revision}
            return display_pose, display_geometry
        if not pose.get('pose_available') or not pose.get('solver_success') or self._geometry_backend is None:
            return display_pose, display_geometry
        cache = self._shape_preview_cache
        if cache is not None and cache[0] == self._shape_preview_revision and 0 <= now - cache[1] < 1. / 30.:
            return cache[2], cache[3]
        params = (self._shape_candidate['shape_profile']['shape_params'] if self._shape_preview == 'candidate'
                  else self._shape_candidate.get('before_shape_params', [0.] * 10))
        try:
            geometry = self._geometry_backend.render_mano(source['mano_pose_rotvec'], params)
        except Exception as exc:
            self._shape_error = f'候选显示失败，当前结果未改变：{exc}'
            return display_pose, display_geometry
        metadata = {**deepcopy(source), 'shape_params': list(params), 'display_only': True,
                    'shape_preview_mode': self._shape_preview, 'shape_preview_revision': self._shape_preview_revision}
        geometry = {**geometry, 'display_only': True,
                    'provenance': {**geometry.get('provenance', {}), 'shape_preview': self._shape_preview}}
        self._shape_preview_cache = (self._shape_preview_revision, now, metadata, geometry)
        return metadata, geometry
