"""URDF (BlueROV2 Heavy + Reach Alpha 5) -> USD via Isaac's importer. Headless."""
import os
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.importer.urdf")
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
H = os.path.dirname(os.path.abspath(__file__))
cfg = URDFImporterConfig(urdf_path=f"{H}/rov_alpha.urdf", usd_path=f"{H}/usd")
out = URDFImporter(cfg).import_urdf()
print("USD_OUT", out, flush=True)
import omni.usd
omni.usd.get_context().open_stage(out)
st = omni.usd.get_context().get_stage()
for p in st.Traverse():
    t = p.GetTypeName()
    if t in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint") or p.HasAPI(__import__("pxr").UsdPhysics.RigidBodyAPI) or p.HasAPI(__import__("pxr").UsdPhysics.ArticulationRootAPI):
        print("PRIM", t, p.GetPath(), flush=True)
app.close()
