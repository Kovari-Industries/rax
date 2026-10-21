"""Lease client for Evo #1230's hardware-loop hold correction, not a torque controller."""
import math
import time
from services import arming


def trial_limit(profile):
    if profile not in ('default', 'attended_peak'):
        raise ValueError('unknown hold profile')
    return 15. if profile == 'attended_peak' else 9.


def check_driver_profile(enabled, overload, slew, profile):
    trial_limit(profile)
    if enabled or overload != (profile == 'attended_peak') or slew != 3.:
        raise RuntimeError('driver hold envelope does not match operator profile')


def check_sample(actuators, now_ns, reference=None, profile='default'):
    limit = trial_limit(profile)
    if len(actuators) != 7:
        raise RuntimeError('incomplete arm telemetry')
    pose = []
    for i, a in enumerate(actuators):
        stamp = a.feedback_stamp.sec*10**9 + a.feedback_stamp.nanosec
        command_stamp = a.command_stamp.sec*10**9 + a.command_stamp.nanosec
        if (not a.feedback_valid or not a.command_valid or a.feedback_faulted
                or a.feedback_stale or a.feedback_missing or a.status != 1
                or not 0 <= now_ns-stamp < 100_000_000
                or not 0 <= now_ns-command_stamp < 100_000_000):
            raise RuntimeError('stale or unhealthy arm telemetry')
        if not all(math.isfinite(x) for x in [a.position, a.velocity, a.command_position,
                                             a.reported_effort, a.rotor_temperature_c, a.mos_temperature_c]):
            raise RuntimeError('nonfinite arm telemetry')
        if abs(a.command_position-a.position) >= .1 or a.rotor_temperature_c >= 50 or a.mos_temperature_c >= 60:
            raise RuntimeError('tracking or temperature guard')
        if i >= 2 and (not math.isfinite(a.command_velocity) or abs(a.command_velocity) > 1e-6):
            raise RuntimeError('command target is moving')
        if i in (2, 3) and abs(a.reported_effort) >= limit:
            raise RuntimeError(f'{limit:g} Nm diagnostic bound reached')
        pose.append(a.command_position)
    if reference is not None and any(abs(x-y) > 1e-6 for x,y in zip(pose,reference)):
        raise RuntimeError('fixed joint target changed')
    return pose



# More than two J4 encoder counts (25 / 65535 rad per count).
MIN_EFFECT_RAD = .001


def summarize_errors(baseline, retained):
    before = sum(abs(x) for x in baseline[-25:]) / len(baseline[-25:])
    after = sum(abs(x) for x in retained[-25:]) / len(retained[-25:])
    meaningful = before >= MIN_EFFECT_RAD
    improved = meaningful and before - after >= max(MIN_EFFECT_RAD, .05 * before)
    return {'ok': improved, 'baseline_abs_error_rad': before,
            'retained_abs_error_rad': after,
            'classification': ('insufficient_baseline_error' if not meaningful else
                               'observed_improvement' if improved else 'no_measurable_improvement')}


def run(config, arm, event_path, profile='default'):
    """One J4 baseline/hold/withdraw observation. Caller owns MCAP and motion exclusion."""
    trial_limit(profile)
    if arm not in ('left', 'right'):
        raise ValueError('arm must be left or right')
    import json
    import threading
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from evo_interfaces.msg import DamiaoCalibrationCommand, DamiaoTelemetry
    from pathlib import Path
    if not arming.read(config.root)['armed']:
        raise RuntimeError('hold correction requires operator arming')
    context=Context();rclpy.init(context=context)
    node=rclpy.create_node('rax_j4_hold_trial',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    latest={};lock=threading.Lock();session=time.time_ns();reference=None
    path=Path(event_path);path.parent.mkdir(parents=True,exist_ok=True)
    output=path.open('a');enabled=False;participated=False
    def event(kind, **data):
        output.write(json.dumps({'timestamp_ns':time.time_ns(),'session_id':session,'arm':arm,'event':kind,**data})+'\n');output.flush()
    def update(key,msg):
        with lock:latest[key]=(time.time_ns(),msg)
    node.create_subscription(DamiaoTelemetry,f'/{arm}_arm/damiao/telemetry',lambda m:update('telemetry',m),qos_profile_sensor_data)
    node.create_subscription(DamiaoCalibrationCommand,f'/{arm}_arm/damiao/calibration_state',lambda m:update('state',m),1)
    pub=node.create_publisher(DamiaoCalibrationCommand,f'/{arm}_arm/damiao/calibration',1)
    client=AsyncParameterClient(node,f'/damiao_diagnostics_{arm}_')
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def wait(future):
        deadline=time.monotonic()+2
        while not future.done() and time.monotonic()<deadline:time.sleep(.01)
        if not future.done():future.cancel();raise RuntimeError('driver parameter request timed out')
        return future.result()
    def parameters(**values):
        result=wait(client.set_parameters_atomically([Parameter(k,value=v) for k,v in values.items()]))
        if result is None or not result.result.successful:raise RuntimeError('driver rejected hold parameters')
    def command(enable):
        pub.publish(DamiaoCalibrationCommand(stamp=node.get_clock().now().to_msg(),session_id=session,enable=enable,torque_nm=[0.,0.]))
    def sample():
        if not arming.read(config.root)['armed']:raise RuntimeError('operator disarmed')
        with lock:value=latest.get('telemetry')
        if value is None or time.time_ns()-value[0]>=100_000_000:raise RuntimeError('telemetry unavailable')
        a=value[1].actuators
        if [x.joint_name for x in a] != [f'{arm}_joint{i}' for i in range(1,8)]:raise RuntimeError('unexpected joint order')
        check_sample(a,time.time_ns(),reference,profile)
        x=a[3]
        event('sample',error_rad=x.command_position-x.position,velocity_rad_s=x.velocity,reported_effort_nm=x.reported_effort,command=x.command_position,measured=x.position)
        return a
    try:
        if not client.wait_for_services(timeout_sec=2):raise RuntimeError('hold driver is not installed')
        params=wait(client.get_parameters(['calibration_enabled','calibration_overload_trial','calibration_hold_slew']))
        if params is None or len(params.values)!=3 or [v.type for v in params.values] != [1,1,3]:raise RuntimeError('hold driver parameters unavailable')
        check_driver_profile(params.values[0].bool_value, params.values[1].bool_value, params.values[2].double_value, profile)
        event('profile',profile=profile,effort_limit_nm=trial_limit(profile))
        time.sleep(.2)
        reference=check_sample(sample(),time.time_ns(),profile=profile)
        event('baseline_start',target=reference)
        baseline=[];end=time.monotonic()+2
        while time.monotonic()<end:
            a=sample()[3];baseline.append(a.command_position-a.position);time.sleep(.02)
        if sum(abs(x) for x in baseline[-25:])/len(baseline[-25:]) < MIN_EFFECT_RAD:
            raise RuntimeError('insufficient baseline error; correction not enabled')
        parameters(calibration_enabled=False);parameters(calibration_hold_joint=4)
        # Mark before enabling so cleanup runs even if the reply is lost.
        participated=True;enabled=True;parameters(calibration_enabled=True)
        event('pulse_start');begin=time.monotonic();seen=False;reason=None
        while time.monotonic()-begin<2.5:
            sample();command(True)
            with lock:state=latest.get('state')
            if state and state[1].session_id==session:
                msg=state[1];stamp=msg.stamp.sec*10**9+msg.stamp.nanosec
                if not 0<=time.time_ns()-stamp<250_000_000:raise RuntimeError('stale hold acknowledgement')
                seen=True;event('hold_state',active=msg.enable,applied=list(msg.torque_nm),reason=msg.cancellation_reason)
                if not msg.enable:
                    reason=msg.cancellation_reason
                    if reason not in ('predicted_target','pulse_timeout'):raise RuntimeError('driver stopped pulse: '+reason)
                    break
            elif time.monotonic()-begin>.5:raise RuntimeError('hold acknowledgement missing')
            time.sleep(.02)
        if not seen or reason is None:raise RuntimeError('hold pulse did not terminate within its bound')
        command(False);parameters(calibration_enabled=False);enabled=False
        event('withdraw');retained=[];quiet_since=None;end=time.monotonic()+2
        while time.monotonic()<end:
            values=sample();a=values[3];retained.append(a.command_position-a.position)
            if all(abs(x.velocity)<.04 for x in values):
                if quiet_since is None:quiet_since=time.monotonic()
            else:quiet_since=None
            time.sleep(.02)
        if quiet_since is None or time.monotonic()-quiet_since<.5:
            raise RuntimeError('measured stop not verified after withdrawal')
        with lock:state=latest.get('state')
        if state is None or time.time_ns()-state[0]>=250_000_000 or state[1].enable or any(abs(x)>1e-9 for x in state[1].torque_nm):
            raise RuntimeError('correction withdrawal not verified')
        result={**summarize_errors(baseline, retained),'paired_acceptance_established':False,'stop_verified':True,'session_id':session,'arm':arm,'joint':4,'pulse_reason':reason,'baseline_error_rad':baseline,'retained_error_rad':retained,'physical_tool_accuracy_verified':False}
        event('complete',result=result);return result
    except BaseException as exc:
        event('failure',reason=str(exc));raise
    finally:
        if participated:command(False)
        if enabled:
            try:parameters(calibration_enabled=False)
            except Exception as e:event('disable_failed',reason=str(e))
        executor.shutdown(timeout_sec=2);thread.join(timeout=2);node.destroy_node();context.shutdown();output.close()
