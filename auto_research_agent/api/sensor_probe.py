"""Read-only Evo sensor receipts for hardware/simulator interface comparison.

No arm, disarm, preview, motion, parameter or calibration call is permitted.
Capture timestamps are retained, not treated as host time without clock proof.
"""
import time


def probe_sensors(context, emit, *, samples=3, include_scene=True, clock=time):
    """Emit each start and completion immediately, retaining partial failures.

    ``emit`` should append one JSON object and flush it. RGB observations and
    scene geometry are separate capabilities. This routine does not interpret
    RGB metadata as depth, infer a clock offset, or qualify sensor accuracy.
    """
    if type(samples) is not int or not 1 <= samples <= 100:
        raise ValueError('Sensor sample count must be 1..100')
    if type(include_scene) is not bool:
        raise ValueError('Scene selection must be boolean')
    rows=[]
    calls=[(-1,'evo_capabilities'),(-1,'evo_status')]
    for index in range(samples):
        calls.append((index,'evo_observe'))
        if include_scene:calls.append((index,'rax_scene'))
    for index,tool in calls:
        start=clock.monotonic()
        row=dict(sample=index,tool=tool,start_monotonic_s=start,
                 start_host_wall_s=clock.time(),phase='started')
        emit(dict(row))
        try:
            row['response']=context.value(tool)
            row['passed']=True
        except Exception as error:
            row.update(passed=False,error=repr(error))
        finally:
            row.update(end_monotonic_s=clock.monotonic(),end_host_wall_s=clock.time(),phase='finished')
            row['call_duration_s']=row['end_monotonic_s']-start
            row['clock_scope']='Host call duration only; capture clock offset and transport age unqualified'
            rows.append(row)
            emit(dict(row))
    return dict(complete=True,passed=all(r['passed'] for r in rows),calls=rows,
                scope='Read-only interface/latency record; no physical action or calibration qualification')
