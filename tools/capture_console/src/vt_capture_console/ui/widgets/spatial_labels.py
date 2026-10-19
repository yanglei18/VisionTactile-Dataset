"""Shared screen-space labels for measured spatial entities."""
from __future__ import annotations

from ...i18n import tr
from ...i18n.labels import tracker_role

from dataclasses import dataclass
import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPainterPath, QPen
from ...i18n.widgets import QApplication

from ...viewmodels.spatial import STATE_LABELS


@dataclass(frozen=True)
class LabelPlacement:
    identifier: str
    center: QPointF
    rect: QRectF
    title: str
    subtitle: str
    color: str
    selected: bool


def _label_fonts(font):
    """Keep inherited font rendering settings, including grayscale antialiasing."""
    title = QFont(font)
    title.setPixelSize(12)
    title.setWeight(QFont.Weight.Medium)
    subtitle = QFont(font)
    subtitle.setPixelSize(10)
    subtitle.setWeight(QFont.Weight.Normal)
    return title, subtitle


def _subtitle(model, entity):
    role = entity.get("role") or entity.get("id", "")
    role = tracker_role(role)
    state = tr('位姿无效') if entity.get("valid") is False else STATE_LABELS.get(entity.get("state"), tr('状态未知'))
    if model.frozen:
        state = tr('冻结 · ') + state
    elif not model.source_available:
        state = tr('数据源不可用')
    # Keep the state first so long external identifiers cannot elide warnings.
    return tr('{v0} / {v1}', v0=state, v1=role) if role else state


def _clamp_rect(rect, bounds):
    return QRectF(min(max(rect.x(), bounds.left()), bounds.right()-rect.width()),
                  min(max(rect.y(), bounds.top()), bounds.bottom()-rect.height()),
                  rect.width(), rect.height())


def _overlap_area(first, second):
    overlap = first.intersected(second)
    return max(0., overlap.width()) * max(0., overlap.height())


def layout_entity_labels(model, projected, viewport_rect):
    """Place (entity, projected origin, visible color) labels without moving poses.

    All distances are logical pixels. Candidate scoring keeps the six-device
    scene readable while remaining bounded and deterministic on each repaint.
    """
    viewport_rect = QRectF(viewport_rect)
    bounds = viewport_rect.adjusted(10., 10., -10., -30.)
    if bounds.width() < 70. or bounds.height() < 44.:
        return []
    visible = [(entity, QPointF(center), color) for entity, center, color in projected
               if math.isfinite(center.x()) and math.isfinite(center.y())
               and viewport_rect.contains(center)]
    title_font, subtitle_font = _label_fonts(QApplication.font())
    title_metrics, subtitle_metrics = QFontMetricsF(title_font), QFontMetricsF(subtitle_font)
    placements = []
    for entity, center, color in visible:
        title = entity.get("label") or entity["id"]
        subtitle = _subtitle(model, entity)
        width = min(175., bounds.width(), max(116., title_metrics.horizontalAdvance(title)+22.,
                                              subtitle_metrics.horizontalAdvance(subtitle)+22.))
        height = 44.
        # Prefer the roomier half of the viewport. Other candidates resolve
        # clusters without adding a force simulation or a new animation timer.
        side = 1. if center.x() < viewport_rect.center().x() else -1.
        candidates = []
        for direction in (side, -side):
            x = center.x()+50. if direction > 0 else center.x()-50.-width
            candidates.extend(QRectF(x, center.y()+dy, width, height)
                              for dy in (-height-12., 12., -height/2., -height-65., 65.))
        candidates.extend((QRectF(center.x()-width/2., center.y()-height-50., width, height),
                           QRectF(center.x()-width/2., center.y()+50., width, height)))

        def score(candidate):
            rect = _clamp_rect(candidate, bounds)
            label_overlap = sum(_overlap_area(rect.adjusted(-5., -5., 5., 5.), other.rect)
                                for other in placements)
            origin_overlap = sum(_overlap_area(rect, QRectF(point.x()-25., point.y()-25., 50., 50.))
                                 for _entity, point, _color in visible)
            displacement = abs(rect.x()-candidate.x()) + abs(rect.y()-candidate.y())
            distance = (rect.center()-center).manhattanLength()
            return label_overlap*20. + origin_overlap*12. + displacement*2. + distance*.02

        rect = _clamp_rect(min(candidates, key=score), bounds)
        placements.append(LabelPlacement(entity["id"], center, rect, title, subtitle, color,
                                         entity["id"] == model.selected_id))
    return placements


def draw_entity_labels(painter, placements):
    """Paint neutral annotation panels with role color confined to origin dots."""
    painter.save()
    painter.setRenderHint(painter.RenderHint.Antialiasing)
    painter.setRenderHint(painter.RenderHint.TextAntialiasing)
    title_font, subtitle_font = _label_fonts(painter.font())
    for placement in placements:
        rect, center = placement.rect, placement.center
        # Connect to the nearer panel edge using a short orthogonal approach.
        if center.x() < rect.left():
            anchor = QPointF(rect.left(), rect.center().y())
            elbow = QPointF(rect.left()-12., anchor.y())
        elif center.x() > rect.right():
            anchor = QPointF(rect.right(), rect.center().y())
            elbow = QPointF(rect.right()+12., anchor.y())
        else:
            anchor = QPointF(rect.center().x(), rect.top() if center.y() < rect.top() else rect.bottom())
            elbow = QPointF(anchor.x(), anchor.y()+(-10. if center.y() < rect.top() else 10.))
        path = QPainterPath(center)
        path.lineTo(elbow)
        path.lineTo(anchor)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(205, 205, 205, 175 if placement.selected else 105), 1.))
        painter.drawPath(path)
        painter.setPen(QPen(QColor(placement.color), 1.2))
        painter.setBrush(QColor("#0b0b0b"))
        painter.drawEllipse(center, 3.1, 3.1)
        painter.setBrush(QColor(placement.color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(center, 1.1, 1.1)

        painter.setBrush(QColor(9, 9, 9, 232 if placement.selected else 211))
        painter.setPen(QPen(QColor(230, 230, 230, 180 if placement.selected else 58), 1.))
        painter.drawRect(rect)
        if placement.selected:
            painter.setPen(QPen(QColor("#ededed"), 1.5))
            painter.drawLine(rect.topLeft(), QPointF(rect.left()+20., rect.top()))

        painter.setFont(title_font)
        painter.setPen(QColor("#f1f1f1" if placement.selected else "#dfdfdf"))
        title_rect = rect.adjusted(10., 5., -10., -21.)
        title = painter.fontMetrics().elidedText(placement.title, Qt.TextElideMode.ElideRight,
                                                 int(title_rect.width()))
        painter.drawText(title_rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, title)
        painter.setFont(subtitle_font)
        painter.setPen(QColor("#a7a7a7"))
        subtitle_rect = rect.adjusted(10., 24., -10., -5.)
        subtitle = painter.fontMetrics().elidedText(placement.subtitle, Qt.TextElideMode.ElideRight,
                                                    int(subtitle_rect.width()))
        painter.drawText(subtitle_rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, subtitle)
    painter.restore()
