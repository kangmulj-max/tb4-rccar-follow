"""patrol node: one capture round = set initial pose → visit waypoints with Nav2 → dwell → save round map.

Params : waypoints (yaml file: list of {x, y, yaw_deg}), initial_pose [x, y, yaw_deg] (optional),
         localization_tol_m=0.3, localization_timeout_s=60 (gate: AMCL must agree with initial_pose before driving),
         dwell_s=2.0, goal_timeout_s=90.0, retries=1, out_dir, mapper_save_srv='scan_mapper/save'
Output : <out_dir>/mission.json — per-waypoint result (reached / failed / skipped), times, totals.
Failure handling: a waypoint that fails or times out is retried `retries` times, then skipped; the round
continues (a partial round is still a valid, flagged result). Ctrl+C cancels the current goal.
"""
import json
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import Trigger


def pose(nav, x, y, yaw_deg, frame='map'):
    p = PoseStamped(); p.header.frame_id = frame; p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator(node_name='patrol')
    P = nav.declare_parameter
    wp_file = P('waypoints', '').value
    init = list(P('initial_pose', [0.0]).value)
    dwell, tmo, retries = P('dwell_s', 2.0).value, P('goal_timeout_s', 90.0).value, P('retries', 1).value
    out_dir = P('out_dir', 'round').value
    srv_name = P('mapper_save_srv', 'scan_mapper/save').value
    wps = yaml.safe_load(open(wp_file, encoding='utf-8'))['waypoints']
    os.makedirs(out_dir, exist_ok=True)
    log = dict(waypoints=[], started=time.time())
    loc_tol = P('localization_tol_m', 0.3).value
    loc_timeout = P('localization_timeout_s', 60.0).value
    amcl = {}
    nav.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: amcl.__setitem__('p', m.pose.pose), 10)
    nav.waitUntilNav2Active(localizer='amcl')
    if len(init) == 3:
        # Localization gate: the initial pose sent the moment AMCL starts can be dropped ("extrapolation into
        # the future"), and driving on a wrong pose ends in recovery behaviours pushing the robot into a wall.
        # Re-send until AMCL reports a pose within loc_tol of the given one, else abort the round (no driving).
        t0 = time.time(); last = 0.0; ok = False
        while time.time() - t0 < loc_timeout:
            if time.time() - last > 3.0:
                nav.setInitialPose(pose(nav, *init)); last = time.time()
            rclpy.spin_once(nav, timeout_sec=0.2)
            p = amcl.get('p')
            if p is not None and math.hypot(p.position.x - init[0], p.position.y - init[1]) <= loc_tol:
                ok = True; break
        log['localization'] = dict(ok=ok, wait_s=round(time.time() - t0, 1))
        if not ok:
            nav.get_logger().error(f'localization did not converge within {loc_timeout} s — round aborted, robot not moved')
            log['aborted'] = 'localization'
            with open(os.path.join(out_dir, 'mission.json'), 'w', encoding='utf-8') as f:
                json.dump(log | dict(round_s=0, reached=0, total=len(wps)), f, indent=1, ensure_ascii=False)
            nav.destroy_node(); rclpy.try_shutdown(); return
        time.sleep(1.0)
    t_round = time.time()
    for i, w in enumerate(wps):
        rec = dict(id=w.get('id', i), x=w['x'], y=w['y'], result='skipped', tries=0, t_s=None)
        for k in range(retries + 1):
            rec['tries'] = k + 1; t0 = time.time()
            nav.goToPose(pose(nav, w['x'], w['y'], w.get('yaw_deg', 0.0)))
            timed_out = False
            while not nav.isTaskComplete():
                if time.time() - t0 > tmo:
                    timed_out = True; nav.cancelTask(); break
                time.sleep(0.1)
            # after a cancel, getResult() can still hold the previous goal's status → never count a timeout as reached
            r = 'TIMEOUT' if timed_out else nav.getResult()
            if r == TaskResult.SUCCEEDED:
                rec.update(result='reached', t_s=round(time.time() - t0, 1)); break
            rec['result'] = 'failed'
            nav.get_logger().warn(f'waypoint {rec["id"]} {r} (try {k + 1})')
        log['waypoints'].append(rec)
        time.sleep(dwell)                                  # stop-and-observe (also lets the mapper see a static view)
        nav.get_logger().info(f'wp {rec["id"]}: {rec["result"]} ({rec["t_s"]} s)')
    log['round_s'] = round(time.time() - t_round, 1)
    log['reached'] = sum(w['result'] == 'reached' for w in log['waypoints']); log['total'] = len(wps)
    cli = nav.create_client(Trigger, srv_name)
    if cli.wait_for_service(timeout_sec=5.0):
        fut = cli.call_async(Trigger.Request()); rclpy.spin_until_future_complete(nav, fut, timeout_sec=30.0)
        log['save'] = fut.result().message if fut.result() else 'no response'
    else:
        log['save'] = f'service {srv_name} not available'
    with open(os.path.join(out_dir, 'mission.json'), 'w', encoding='utf-8') as f:
        json.dump(log, f, indent=1, ensure_ascii=False)
    nav.get_logger().info(f'round done: {log["reached"]}/{log["total"]} waypoints, {log["round_s"]} s')
    nav.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
