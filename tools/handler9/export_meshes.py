#!/usr/bin/env python3
"""Export the CAD parts of 'Bambino Kinematic Schematics.step' to tests/fixtures/handler9/meshes/ (one STL per part,
in its link frame, CAD length units) plus meshes/boxes.json (collision box per part). Needs cadquery-ocp + trimesh.
Usage: python export_meshes.py path/to/model.step"""
import sys, json, pathlib, tempfile
import numpy as np, trimesh
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import gen_handler9_urdf as g
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

# CAD product -> part name. Handler_1 = the non-mirror HandlingGripper1 (x < 0); blue = the mirror CenterPlate gripper (x < 0).
NAMES = {
    "HG_Plate+ExtentionAxis": "handler_1_plate", "StaticGripper": "handler_1_static",
    "ActiveGriper - 95 LegthUnits Extention": "handler_1_active",
    "HG_Plate+ExtentionAxis(Mirror)": "handler_2_plate", "StaticGripper(Mirror)": "handler_2_static",
    "ActiveGriper - 95 LegthUnits Extention(Mirror)": "handler_2_active",
    "CP_Plate+ExtentionAxis": "main_clamp_plate",
    "Linear Translation(Mirror)": "main_clamp_blue_translation", "Axial Rotation_+-40Deg(Mirror)": "main_clamp_blue_rotation",
    "Linear Translation": "main_clamp_yellow_translation", "Axial Rotation_+-40Deg": "main_clamp_yellow_rotation",
}

def solids(step):
    """(product name or None, world mesh in CAD coords) per solid, via the XCAF assembly tree."""
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence, TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopLoc import TopLoc_Location
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    r = STEPCAFControl_Reader(); r.SetNameMode(True); assert r.ReadFile(step) == 1; r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    def nm(l):
        a = TDataStd_Name(); return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), a) else None
    res = []
    def walk(l, loc, parent):
        if st.IsReference_s(l):
            rl = TDF_Label(); st.GetReferredShape_s(l, rl); walk(rl, loc.Multiplied(st.GetLocation_s(l)), parent); return
        if st.IsAssembly_s(l):
            seq = TDF_LabelSequence(); st.GetComponents_s(l, seq)
            for i in range(1, seq.Length() + 1): walk(seq.Value(i), loc, nm(l))
            return
        shp = st.GetShape_s(l).Moved(loc); b = Bnd_Box(); BRepBndLib.Add_s(shp, b)
        if b.IsVoid(): return
        BRepMesh_IncrementalMesh(shp, 0.5, False, 0.3, True)
        with tempfile.NamedTemporaryFile(suffix=".stl") as t:
            StlAPI_Writer().Write(shp, t.name); res.append((nm(l) or parent, trimesh.load(t.name)))
    roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1): walk(roots.Value(i), TopLoc_Location(), None)
    return res

out = g.HERE / "meshes"; out.mkdir(parents=True, exist_ok=True)
boxes = {}
for name, mesh in solids(sys.argv[1]):
    v = mesh.vertices[:, [0, 2, 1]] * [1, -1, 1]   # CAD Y-up -> Motus Z-up: (x, y, z) -> (x, -z, y)
    # The beam and the cane export as unnamed 'SOLID's: the beam is the long one below the beam top.
    part = NAMES.get(name) or ("backbone" if np.ptp(v[:, 0]) > 900 and v[:, 2].max() <= 80.5 else None)
    if part is None:
        print("skip", name, v.min(0).round(1), v.max(0).round(1)); continue   # the cane is not part of the robot
    v = v - np.array(g.FRAMES[g.PARTS[part]])
    trimesh.Trimesh(v, mesh.faces).export(out / f"{part}.stl")
    lo, hi = v.min(0), v.max(0); boxes[part] = [((lo + hi) / 2).round(2).tolist(), (hi - lo).round(2).tolist()]
    print(part, len(mesh.faces), "faces")
missing = set(g.PARTS) - set(boxes)
assert not missing, f"missing parts: {missing}"
(out / "boxes.json").write_text(json.dumps(boxes, indent=1, sort_keys=True))
