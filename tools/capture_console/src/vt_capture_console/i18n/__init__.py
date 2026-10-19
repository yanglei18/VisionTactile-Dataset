"""Local presentation translations; sensor identities and IPC data stay unchanged.

``tr`` retains the source and format arguments so existing widgets can be
retranslated without rebuilding the interface or touching a backend. This
module is deliberately independent of Qt for the pure presentation models.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

LANGUAGES = (('zh_CN', '中文'), ('en', 'English'))
_language = 'zh_CN'
_catalog = None
_settings = None


def normalize_language(value):
    aliases = {'zh': 'zh_CN', 'zh-cn': 'zh_CN', 'zh_cn': 'zh_CN',
               'en': 'en', 'en-us': 'en', 'en_us': 'en', 'en-gb': 'en'}
    try:
        return aliases[str(value).lower()]
    except KeyError:
        raise ValueError(f'Unsupported interface language: {value}') from None


def language():
    return _language


def catalog():
    global _catalog
    if _catalog is None:
        _catalog = {}
        for path in sorted(Path(__file__).parent.glob('catalog_*.json')):
            entries = json.loads(path.read_text(encoding='utf-8'))
            overlap = _catalog.keys() & entries.keys()
            if overlap:
                raise ValueError(f'Duplicate translations in {path.name}: {sorted(overlap)}')
            _catalog.update(entries)
    return _catalog


def render(value):
    return value.render() if isinstance(value, LocalizedText) else value


class LocalizedText(str):
    """A display string retaining its translation recipe, never a protocol ID."""
    def __new__(cls, renderer):
        instance = super().__new__(cls, renderer())
        instance._renderer = renderer
        return instance

    def render(self):
        return self._renderer()

    def __str__(self):
        return self.render()

    def __copy__(self):
        return self

    def __deepcopy__(self, memo):
        # Translation recipes are presentation values, shared like immutable
        # strings. They never own or modify a backend/sensor snapshot.
        memo[id(self)] = self
        return self

    def __format__(self, spec):
        return format(self.render(), spec)

    def __add__(self, other):
        if not isinstance(other, str):
            return NotImplemented
        return LocalizedText(lambda: self.render() + render(other))

    def __radd__(self, other):
        if not isinstance(other, str):
            return NotImplemented
        return LocalizedText(lambda: render(other) + self.render())

    def __getitem__(self, key):
        return LocalizedText(lambda: self.render()[key])

    def format(self, *args, **kwargs):
        return LocalizedText(lambda: self.render().format(
            *(render(v) for v in args), **{k: render(v) for k, v in kwargs.items()}))

    def join(self, values):
        values = tuple(values)
        return LocalizedText(lambda: self.render().join(render(v) for v in values))


def tr(source, **arguments):
    """Translate an authored message before formatting user/sensor values."""
    if isinstance(source, LocalizedText) and not arguments:
        return source
    def translated():
        template = catalog().get(source, source) if _language == 'en' else source
        if arguments:
            return template.format(**{key: render(value) for key, value in arguments.items()})
        return template
    return LocalizedText(translated)


def initialize(settings=None, *, override=None):
    global _settings
    if settings is None:
        from PySide6.QtCore import QSettings
        settings = QSettings(QSettings.Format.IniFormat, QSettings.Scope.UserScope,
                             'visiontactile', 'capture-console')
    _settings = settings
    saved = settings.value('ui/language', 'zh_CN') if override is None else override
    try:
        code = normalize_language(saved)
    except ValueError:
        code = 'zh_CN'
    set_language(code, persist=False)


def set_language(value, *, persist=True):
    global _language
    code = normalize_language(value)
    if persist and _settings is not None:
        _settings.setValue('ui/language', code)
        _settings.sync()
    changed = code != _language
    _language = code
    module = sys.modules.get(__name__ + '.widgets')
    if module is not None:
        module.retranslate_all(changed=changed)
