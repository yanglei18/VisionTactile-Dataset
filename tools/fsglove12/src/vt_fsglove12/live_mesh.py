"""Fixed-topology smooth mesh data for the live MANO surface.

Faces, face-to-vertex adjacency and colors survive pose updates. Vertex normals
retain pyqtgraph's area-weighted smooth shading, but use vectorized accumulation
instead of a Python loop per vertex. This module performs no OpenGL drawing.

Usage::

    data = FixedTopologyMeshData(vertexes=vertices, faces=faces, vertexColors=colors)
    item = GLMeshItem(meshdata=data, smooth=True, ...)
    data.update_vertices(new_vertices)
    item.meshDataChanged()  # invalidate the item's references and schedule paint

Do not call setMeshData(vertexes=..., faces=...) each frame: that creates a new
MeshData and discards these caches. The display basis belongs to the caller.
"""
from __future__ import annotations

import numpy as np
from pyqtgraph.opengl import GLMeshItem, MeshData


class JointVisibilityMeshItem(GLMeshItem):
    """Draw an explicit triangle subset without mutating cached mesh topology.

    Unlike alpha-zero placeholders this cannot show or depth-occlude a fake
    straight joint. Only draw indices change at visual quality transitions;
    current vertices, full normals, and the fixed MANO topology stay intact.
    """
    def __init__(self, **kwargs):
        self._visible_faces = None
        super().__init__(**kwargs)

    def set_face_visibility(self, mask):
        faces = self.opts["meshdata"].faces()
        mask = np.asarray(mask, dtype=bool)
        if mask.shape != (len(faces),):
            raise ValueError("face visibility must match triangle count")
        self._visible_faces = faces if mask.all() else np.ascontiguousarray(faces[mask])
        self.meshDataChanged()

    def parseMeshData(self):
        super().parseMeshData()
        if self._visible_faces is not None:
            if self.faces is None:
                raise ValueError("joint visibility requires indexed smooth mesh data")
            self.faces = self._visible_faces


def _vertices(value, count=None):
    with np.errstate(over="ignore", invalid="ignore"):
        vertices = np.array(value, dtype=np.float32, order="C", copy=True)
    if (vertices.ndim != 2 or vertices.shape[1:] != (3,) or not len(vertices)
            or (count is not None and len(vertices) != count)
            or not np.isfinite(vertices).all()):
        raise ValueError("vertexes must be a finite (N, 3) array with the fixed vertex count")
    return vertices


class FixedTopologyMeshData(MeshData):
    """Vertex updates with immutable triangle topology and cached smooth normals.

    The canonical representation always has one entry per vertex plus triangle
    indices. Querying a cached face-indexed view does not change that format.
    Degenerate/isolated vertices have zero normals rather than NaNs. Colors are
    never recalculated from the changing pose, preserving palmar/dorsal styling.
    """

    def __init__(self, vertexes, faces, vertexColors=None):
        vertices = _vertices(vertexes)
        triangles = np.asarray(faces)
        if (triangles.ndim != 2 or triangles.shape[1:] != (3,)
                or triangles.dtype.kind not in "iu"
                or (triangles.size and (triangles.min() < 0 or triangles.max() >= len(vertices)))):
            raise ValueError("faces must be integer (F, 3) indices within the vertex array")
        triangles = np.array(triangles, dtype=np.uint32, order="C", copy=True)
        colors = None
        if vertexColors is not None:
            colors = np.array(vertexColors, dtype=np.float32, order="C", copy=True)
            if colors.shape not in ((len(vertices), 3), (len(vertices), 4)) or not np.isfinite(colors).all():
                raise ValueError("vertexColors must be a finite (N, 3) or (N, 4) array")
        self._topology_locked = False
        self._vertex_count = len(vertices)
        super().__init__(vertexes=vertices, faces=triangles, vertexColors=colors)
        self._faces.setflags(write=False)
        self._corner_vertices = self._faces.ravel()
        self._corner_faces = np.repeat(np.arange(len(triangles), dtype=np.intp), 3)
        self._corner_faces.setflags(write=False)
        # Build the public adjacency view once, never once per displayed pose.
        self.vertexFaces()
        self._topology_locked = True

    def setFaces(self, faces):
        if self._topology_locked:
            raise ValueError("triangle topology is fixed; create new mesh data to change faces")
        super().setFaces(faces)

    def setVertexes(self, verts=None, indexed=None, resetNormals=True):
        if self._topology_locked:
            if indexed is not None:
                raise ValueError("fixed topology requires non-face-indexed vertices")
            self.update_vertices(verts)
            return
        super().setVertexes(verts, indexed=indexed, resetNormals=resetNormals)

    def update_vertices(self, vertexes):
        """Replace positions and invalidate geometric caches, retaining topology/colors.

        Validation completes before mutation, so a malformed update cannot leave
        the currently displayed mesh half changed. The GLMeshItem owner must
        subsequently call meshDataChanged(); there are no GL calls here.
        """
        vertices = _vertices(vertexes, self._vertex_count)
        self._vertexes = vertices
        self._vertexesIndexedByFaces = None
        self._vertexesIndexedByEdges = None
        self.resetNormals()

    def hasFaceIndexedData(self):
        # MeshData's base implementation confuses a cached face view with the
        # canonical format. Our permanent triangle indices remain valid even
        # after faceNormals()/vertexes(indexed='faces') were inspected.
        return False

    def vertexNormals(self, indexed=None):
        if indexed not in (None, "faces"):
            raise ValueError("normal indexing must be None or 'faces'")
        if self._vertexNormals is None:
            face_normals = self.faceNormals()
            # Each corner contributes its (unnormalized) face cross product:
            # exactly the area weighting used by pyqtgraph's original method.
            normals = np.stack([
                np.bincount(self._corner_vertices,
                            weights=face_normals[self._corner_faces, axis],
                            minlength=self._vertex_count)
                for axis in range(3)
            ], axis=1).astype(np.float32)
            lengths = np.linalg.norm(normals, axis=1, keepdims=True)
            np.divide(normals, lengths, out=normals, where=lengths > 0)
            self._vertexNormals = normals
        if indexed is None:
            return self._vertexNormals
        if self._vertexNormalsIndexedByFaces is None:
            self._vertexNormalsIndexedByFaces = self._vertexNormals[self._faces]
        return self._vertexNormalsIndexedByFaces
