"""Persistent caches for cuRobo's runtime-compiled CUDA kernels.

cuRobo v0.8.0 compiles its CUDA kernels at runtime with NVRTC (``cuda.core`` backend, the
supported path; the deprecated pybind extension build is incomplete in this release) and
keeps them only in process memory. Warp kernels are JIT-compiled into a disk cache.
Neither needs nvcc or network access, but recompiling in every new process costs seconds.

``install()`` (called before cuRobo is imported) adds two read-through caches:

* cuRobo NVRTC kernels: cubins keyed exactly as cuRobo keys them (SHA-256 of the kernel
  source, kernel name, flags and GPU arch). Lookup order: the read-only cache prebuilt at
  install time inside this package (``prebuilt/curobo``), then a per-user cache
  (``$HUMANOID_G1_CACHE`` or ``$XDG_CACHE_HOME/humanoid-g1`` or ``~/.cache/humanoid-g1``). Misses
  are compiled with NVRTC as usual and written to the per-user cache.
* Warp: the per-user warp kernel directory is seeded from ``prebuilt/warp-<version>`` once.
* PyTorch's jiterator kernels (cuRobo's samplers use ``torch.erfinv``): ``PYTORCH_KERNEL_CACHE_PATH``
  points to a per-user directory seeded from ``prebuilt/torch-kernels`` (unless already set).

If anything about these caches fails, cuRobo's own in-memory compilation is used unchanged.
"""
from __future__ import annotations

import json
import os
import shutil
from pathlib import Path

PREBUILT = Path(__file__).resolve().parent / "prebuilt"
_INSTALLED = False
STATS = {"hits_prebuilt": 0, "hits_user": 0, "compiled": 0, "errors": 0}


def user_cache_dir() -> Path:
    """Writable per-user cache: $HUMANOID_G1_CACHE, $XDG_CACHE_HOME/humanoid-g1, ~/.cache/humanoid-g1,
    else <tmp>/humanoid-g1-<uid>."""
    if os.environ.get("HUMANOID_G1_CACHE"):
        return Path(os.environ["HUMANOID_G1_CACHE"])
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    candidate = Path(base) / "humanoid-g1"
    if _writable(candidate):
        return candidate
    import tempfile
    return Path(tempfile.gettempdir()) / f"humanoid-g1-{os.getuid()}"


def _writable(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / f".probe-{os.getpid()}"
        probe.write_bytes(b"")
        probe.unlink()
        return True
    except OSError:
        return False


def _seed(prebuilt: Path, target: Path):
    """Copy a prebuilt cache directory into a writable one, once."""
    if prebuilt.is_dir() and not (target / ".seeded").exists():
        for item in prebuilt.iterdir():
            destination = target / item.name
            if not destination.exists():
                if item.is_dir():
                    shutil.copytree(item, destination)
                else:
                    shutil.copy2(item, destination)
        (target / ".seeded").write_text("seeded from " + str(prebuilt) + "\n")


def _setup_warp(target_root: Path | None):
    try:
        import warp as wp
    except Exception:
        return None
    version = getattr(wp.config, "version", "unknown")
    if target_root is None:
        return None
    target = target_root / f"warp-{version}"
    if not _writable(target):
        return None
    _seed(PREBUILT / f"warp-{version}", target)
    wp.config.kernel_cache_dir = str(target)
    return target


def _setup_torch_kernels(target: Path, prebuilt: Path | None):
    if "PYTORCH_KERNEL_CACHE_PATH" in os.environ or not _writable(target):
        return
    if prebuilt is not None:
        _seed(prebuilt, target)
    os.environ["PYTORCH_KERNEL_CACHE_PATH"] = str(target)


def install(write_prebuilt: bool = False):
    """Install the caches. ``write_prebuilt`` is used by the installer only."""
    global _INSTALLED
    if _INSTALLED:
        return
    _INSTALLED = True
    if write_prebuilt:
        user = PREBUILT
        PREBUILT.mkdir(parents=True, exist_ok=True)
        _setup_warp_prebuilt()
        _setup_torch_kernels(PREBUILT / "torch-kernels", None)
    else:
        user = user_cache_dir()
        _setup_warp(user if _writable(user) else None)
        _setup_torch_kernels(user / "torch-kernels", PREBUILT / "torch-kernels")
    try:
        _patch_curobo(PREBUILT / "curobo", user / "curobo" if _writable(user) else None)
    except Exception as error:  # never break cuRobo because of the cache
        STATS["errors"] += 1
        STATS["patch_error"] = repr(error)


def _setup_warp_prebuilt():
    try:
        import warp as wp
    except Exception:
        return
    target = PREBUILT / f"warp-{wp.config.version}"
    target.mkdir(parents=True, exist_ok=True)
    wp.config.kernel_cache_dir = str(target)


def _patch_curobo(prebuilt: Path, user: Path | None):
    from curobo._src.curobolib.backends.cuda_core_backend import kernel_cache as kc
    from curobo._src.util.logging import log_info

    cls = kc.CudaCoreKernelCache
    if getattr(cls, "_g1_curobo_patched", False):
        return
    original = cls.get_or_compile_kernel

    def load(directory: Path, key: str, kernel_name: str):
        from cuda.core import ObjectCode
        cubin = directory / f"{key}.cubin"
        meta = directory / f"{key}.json"
        if not cubin.is_file() or not meta.is_file():
            return None
        info = json.loads(meta.read_text())
        if info.get("kernel_name") != kernel_name:
            return None
        mapping = {k: v.encode() if isinstance(v, str) else v for k, v in info["symbol_mapping"].items()}
        code = ObjectCode.from_cubin(cubin.read_bytes(), symbol_mapping=mapping)
        return code.get_kernel(kernel_name)

    def get_or_compile_kernel(self, source_files, kernel_name, include_dirs, compile_flags):
        self.initialize()
        key = self.get_kernel_hash(source_files, kernel_name, compile_flags)
        if key in self.compiled_kernels:
            return self.compiled_kernels[key]
        kernel = None
        for directory, stat in ((prebuilt, "hits_prebuilt"), (user, "hits_user")):
            if directory is None:
                continue
            try:
                kernel = load(directory, key, kernel_name)
            except Exception:
                kernel = None
            if kernel is not None:
                STATS[stat] += 1
                break
        if kernel is None:
            kernel = compile_and_store(self, source_files, kernel_name, include_dirs, compile_flags, key)
        if kernel is None:
            return original(self, source_files, kernel_name, include_dirs, compile_flags)
        self.compiled_kernels[key] = kernel
        import torch
        torch.cuda.synchronize()
        return kernel

    def compile_and_store(self, source_files, kernel_name, include_dirs, compile_flags, key):
        if user is None:
            return None
        from cuda.core import Program, ProgramOptions
        from cuda import pathfinder
        source = self._read_sources(source_files, include_dirs)
        include_paths = [str(p) for p in include_dirs]
        cuda_include = pathfinder.find_nvidia_header_directory("nvrtc")
        if cuda_include and os.path.isdir(cuda_include):
            include_paths.append(cuda_include)
        options = ProgramOptions(std="c++17", arch=self.arch, include_path=include_paths, ftz=True,
                                 fma=True, prec_div=False, prec_sqrt=False, lineinfo=True,
                                 device_code_optimize=True)
        try:
            program = Program(source, code_type="c++", options=options)
            module = program.compile("cubin", name_expressions=(kernel_name,))
            mapping = dict(getattr(module, "symbol_mapping", None) or getattr(module, "_sym_map", None) or {})
            payload = module.code if isinstance(module.code, (bytes, bytearray)) else None
            kernel = module.get_kernel(kernel_name)
        except Exception:
            STATS["errors"] += 1
            return None
        STATS["compiled"] += 1
        if payload is not None and kernel_name in mapping:
            try:
                user.mkdir(parents=True, exist_ok=True)
                tmp = user / f".{key}.{os.getpid()}"
                tmp.write_bytes(bytes(payload))
                os.replace(tmp, user / f"{key}.cubin")
                lowered = mapping[kernel_name]
                (user / f"{key}.json").write_text(json.dumps({
                    "kernel_name": kernel_name, "arch": self.arch,
                    "symbol_mapping": {kernel_name: lowered.decode() if isinstance(lowered, bytes) else lowered}}))
            except OSError:
                STATS["errors"] += 1
        log_info(f"G1 planner compiled and cached kernel {kernel_name}")
        return kernel

    cls.get_or_compile_kernel = get_or_compile_kernel
    cls._g1_curobo_patched = True
