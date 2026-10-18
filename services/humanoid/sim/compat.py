"""Process-local Isaac Sim 6.1 / Kit compatibility for the pinned HumanoidArena sources.

Both shims are installed before the upstream task modules are imported and leave shared
packages and upstream files unchanged:

* the IsaacLab PreviewSurface shader call follows the Kit 110 ``CreateShaderPrimFromSdrCommand``
  rename (``name`` -> ``prim_name``);
* the optional hardware DDS side channel of the Arena observation functions is disabled: no
  robot hardware exists, and observations, joints, physics and the controller are unaffected.
"""
import inspect
import sys
import types

def adapt_shader_command(command, source_type):
    """Preserve the requested shader name/identifier across the Kit API rename."""
    parameters = inspect.signature(command).parameters
    if 'name' in parameters:
        return command
    if 'prim_name' not in parameters:
        raise RuntimeError('Unsupported CreateShaderPrimFromSdrCommand signature')

    def compatible(*args, name=None, **kwargs):
        if name is not None:
            kwargs['prim_name'] = name
        # New Kit defaults to MDL; UsdPreviewSurface is registered under its own
        # source type. Ask the installed registry instead of substituting a shader.
        if kwargs.get('identifier') == 'UsdPreviewSurface' and 'node_type' in parameters:
            kwargs.setdefault('node_type', source_type())
        return command(*args, **kwargs)

    return compatible


def install_material_compat():
    from isaaclab.sim.spawners.materials import visual_materials
    from pxr import Sdr
    original = visual_materials.CreateShaderPrimFromSdrCommand

    def source_type():
        node = Sdr.Registry().GetShaderNodeByIdentifier('UsdPreviewSurface')
        if node is None:
            raise RuntimeError('Installed Sdr registry lacks UsdPreviewSurface')
        return str(node.GetSourceType())

    adapted = adapt_shader_command(original, source_type)
    if adapted is original:
        return []
    visual_materials.CreateShaderPrimFromSdrCommand = adapted
    receipt = {'component': 'IsaacLab PreviewSurface shader creation',
               'scope': 'process-local call adapter; shared packages and upstream files unchanged',
               'keyword_mapping': {'name': 'prim_name'},
               'shader_identifier': 'UsdPreviewSurface', 'node_type': source_type()}
    print('[humanoid compatibility]', receipt, flush=True)
    return [receipt]


class DisabledHardwareDDS:
    def get_object(self, name):
        return None

    def register_object(self, *args, **kwargs):
        raise RuntimeError("Hardware DDS is disabled in the local HumanoidArena simulator")


def install_local_transport():
    if 'dds.dds_master' in sys.modules:
        raise RuntimeError('Install the local transport before importing Arena tasks')
    module = types.ModuleType('dds.dds_master')
    module.dds_manager = DisabledHardwareDDS()
    sys.modules['dds.dds_master'] = module
    print('[humanoid compatibility] Hardware DDS disabled; physics uses local Arena provider', flush=True)
