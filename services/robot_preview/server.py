"""Launch a separate robot/camera-only Isaac Sim preview service."""

from __future__ import annotations

import argparse
import traceback
from pathlib import Path


def main():
    import cv2  # noqa: F401 - load before Isaac extension dependency paths
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robodojo-root", type=Path, required=True)
    parser.add_argument("--robot-model", default="dual_arx_x5")
    parser.add_argument("--camera-model", default="robodojo_gemini_wrist_v1")
    parser.add_argument("--port", type=int, default=0)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.headless = True
    args.enable_cameras = True
    app = AppLauncher(args).app
    environment = session = None
    try:
        from services.simulator.rpc import serve

        from .environment import RobotOnlyEnvironment, build_configs
        from .session import RobotPreviewSession

        root = args.robodojo_root.resolve()
        sim_config, robot_config, camera_config = build_configs(
            root, robot_model=args.robot_model, camera_model=args.camera_model
        )
        environment = RobotOnlyEnvironment(sim_config, robot_config, camera_config, app)
        environment.start()
        session = RobotPreviewSession(
            environment,
            robot_model=args.robot_model,
            camera_model=args.camera_model,
        )
        serve(session, args.port)
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        if session is not None:
            session.close()
        elif environment is not None:
            environment.close()
        app.close()


if __name__ == "__main__":
    main()
