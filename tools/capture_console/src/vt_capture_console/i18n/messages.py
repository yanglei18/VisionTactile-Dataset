"""Translate complete, authored backend diagnostics at a display boundary.

This module never changes IPC payloads, exceptions, logs, or saved data. Call
``message`` on a diagnostic field when displaying it, not on a user-defined
label, trial name, or path. Template captures are already formatted values:
they remain opaque, including Chinese text, quotes, numbers, and braces.

Only a finite allowlist of complete messages/templates is recognized, including
their English forms and explicitly registered protocol reason codes. Literal
splitting is deterministic and bounded; there is no regex or word replacement.
Ambiguous translations/template boundaries and unknown/oversized messages pass
through. Diagnostic catalogs are presentation resources, never protocol data.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from itertools import islice
import json
from pathlib import Path
from string import Formatter

from . import LocalizedText, catalog, language


MAX_MESSAGE_LENGTH = 4096
MAX_SOURCES = 1024
MAX_TEMPLATES = 160
MAX_ALIASES = 1536
MAX_TEMPLATE_FIELDS = 4
MAX_CACHED_MESSAGES = 256
MAX_RESOURCE_FILES = 16
MAX_RESOURCE_BYTES = 256 * 1024
_FORMATTER = Formatter()


@dataclass(frozen=True)
class _Template:
    source: str
    literals: tuple[str, ...]
    fields: tuple[str, ...]

    def match(self, text: str):
        prefix, suffix = self.literals[0], self.literals[-1]
        if not text.startswith(prefix) or not text.endswith(suffix):
            return None
        end = len(text) - len(suffix) if suffix else len(text)
        if end < len(prefix):
            return None
        remaining = text[len(prefix):end]
        values = []
        for delimiter in self.literals[1:-1]:
            position = remaining.find(delimiter)
            # More than one possible boundary could split opaque user data.
            # Passing the entire message through is safer than guessing.
            if position < 0 or position != remaining.rfind(delimiter):
                return None
            values.append(remaining[:position])
            remaining = remaining[position + len(delimiter):]
        values.append(remaining)
        captures = {}
        for field, value in zip(self.fields, values):
            if field in captures and captures[field] != value:
                return None
            captures[field] = value
        return tuple(captures.items())


def _template(source: str):
    literals, fields = [""], []
    try:
        for literal, field, spec, conversion in _FORMATTER.parse(source):
            literals[-1] += literal
            if field is not None:
                # Attribute/index expressions and nested format fields are not
                # needed by the authored diagnostics and are not evaluated.
                if not field.isidentifier() or "{" in spec or "}" in spec:
                    return None
                fields.append(field)
                literals.append("")
    except ValueError:
        return None
    if (not 1 <= len(fields) <= MAX_TEMPLATE_FIELDS
            or any(not part for part in literals[1:-1])
            or sum(map(len, literals)) < 4):
        return None
    return _Template(source, tuple(literals), tuple(fields))


def _read_resource(path):
    with path.open("rb") as stream:
        payload = stream.read(MAX_RESOURCE_BYTES + 1)
    if len(payload) > MAX_RESOURCE_BYTES:
        raise ValueError("Diagnostic translation resource exceeds its size limit")
    return json.loads(payload)


def _resource_paths(pattern):
    paths = tuple(islice(Path(__file__).parent.glob(pattern), MAX_RESOURCE_FILES + 1))
    if len(paths) > MAX_RESOURCE_FILES:
        raise ValueError("Too many diagnostic translation resource files")
    return tuple(sorted(paths))


def _load_sources():
    path = Path(__file__).with_name("diagnostic_sources.json")
    sources = tuple(_read_resource(path))
    # Dedicated diagnostic catalogs register their canonical display sources;
    # ordinary UI catalogs are never treated as diagnostic allowlists.
    return sources + tuple(source for path in _resource_paths("catalog_diagnostics_*.json")
                           for source in _read_resource(path))


@lru_cache(maxsize=1)
def _index():
    sources = _load_sources()
    if len(sources) > MAX_SOURCES or any(
            not isinstance(source, str) or len(source) > MAX_MESSAGE_LENGTH
            for source in sources):
        raise ValueError("Diagnostic translation allowlist exceeds its limits")
    templates = tuple(candidate for source in sources
                      if (candidate := _template(source)) is not None)
    if len(templates) > MAX_TEMPLATES:
        raise ValueError("Too many diagnostic translation templates")
    # Prefer the most specific complete template if fixed prose overlaps.
    templates = tuple(sorted(templates,
                             key=lambda value: (-sum(map(len, value.literals)), value.source)))
    return frozenset(sources), templates


@lru_cache(maxsize=1)
def _alias_index(sources):
    """Recognize authored English messages and explicit protocol reason codes.

    Ambiguous English translations are omitted. Captured values remain opaque;
    nested diagnostic reasons must be registered as complete aliases.
    """
    aliases, ambiguous = {}, set()
    for source in sources:
        english = catalog().get(source)
        if not english:
            continue
        if english in aliases and aliases[english] != source:
            ambiguous.add(english)
        aliases[english] = source
    for english in ambiguous:
        aliases.pop(english)
    explicit = {}
    for path in _resource_paths("diagnostic_aliases_*.json"):
        entries = _read_resource(path)
        if explicit.keys() & entries.keys():
            raise ValueError("Duplicate diagnostic translation aliases")
        explicit.update(entries)
    aliases.update({alias: source for alias, source in explicit.items() if source in sources})
    if len(aliases) > MAX_ALIASES or any(len(value) > MAX_MESSAGE_LENGTH for value in aliases):
        raise ValueError("Diagnostic translation aliases exceed their limits")
    templates = tuple((candidate, source) for alias, source in aliases.items()
                      if (candidate := _template(alias)) is not None)
    if len(templates) > MAX_TEMPLATES:
        raise ValueError("Too many diagnostic alias templates")
    templates = tuple(sorted(templates, key=lambda item: (
        -sum(map(len, item[0].literals)), item[0].source)))
    return aliases, templates


@lru_cache(maxsize=MAX_CACHED_MESSAGES)
def _resolve(text: str):
    exact, templates = _index()
    if text in exact:
        return text, None
    aliases, alias_templates = _alias_index(exact)
    if text in aliases:
        return aliases[text], None
    for template in templates:
        captures = template.match(text)
        if captures is not None:
            return template.source, captures
    for template, source in alias_templates:
        captures = template.match(text)
        if captures is not None:
            return source, captures
    return None


def _render_message(text: str):
    if len(text) > MAX_MESSAGE_LENGTH:
        return text
    resolved = _resolve(text)
    if resolved is None:
        return text
    source, captures = resolved
    translated = catalog().get(source) if language() == "en" else source
    if translated is None:
        return text
    if captures is None:
        return translated
    values = dict(captures)
    try:
        parts = tuple(_FORMATTER.parse(translated))
        if {field for _, field, _, _ in parts if field is not None} != values.keys():
            return text
        # Do not call format/convert_field: captures already include source
        # formatting (e.g. .1f or !r), so doing it again would corrupt values.
        return "".join(literal + (values[field] if field is not None else "")
                       for literal, field, _, _ in parts)
    except (KeyError, ValueError):
        return text


def message(text: str) -> LocalizedText:
    """Retain the untouched source diagnostic and render in the current language.

    Pass only authored diagnostic fields. In particular, an arbitrary user
    label matching a known phrase must not be sent through this function.
    """
    if isinstance(text, LocalizedText):
        return text
    if not isinstance(text, str):
        raise TypeError("Diagnostic display text must be a string")
    return LocalizedText(lambda: _render_message(text))


def exception_message(error: BaseException) -> LocalizedText:
    """Keep an authored exception's recipe, including localized field values.

    Converting such an exception to str first loses nested translations. Plain
    exceptions still use the bounded diagnostic allowlist and preserve unknown
    OS messages, paths and other user-owned values.
    """
    if len(error.args) == 1 and isinstance(error.args[0], LocalizedText):
        return error.args[0]
    return message(str(error))
