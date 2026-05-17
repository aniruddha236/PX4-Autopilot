#!/usr/bin/env python3
"""
Repeatable MAVSDK offboard figure-8 maneuver for PX4 SIH/SITL.

Run PX4 first, for example:
  make px4_sitl_sih sihsim_hex

Then run:
  python3 Tools/control_allocation/figure8_offboard.py --radius 8 --period 8 --laps 4 --altitude 5

Use the same arguments for CA_METHOD=0, 1, and 3 logs.
"""

import argparse
import asyncio
import math
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def wait_connected(drone: System) -> None:
    print("Waiting for vehicle connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            return


async def wait_ready(drone: System) -> None:
    print("Waiting for global position estimate...")

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            return


async def wait_position_reached(drone: System, north: float, east: float, down: float, radius: float,
                                timeout_s: float) -> bool:
    start = time.monotonic()

    async for position in drone.telemetry.position_velocity_ned():
        dn = position.position.north_m - north
        de = position.position.east_m - east
        dd = position.position.down_m - down

        if math.sqrt(dn * dn + de * de + dd * dd) <= radius:
            return True

        if time.monotonic() - start > timeout_s:
            return False


async def stream_position(drone: System, north: float, east: float, down: float, yaw_deg: float,
                          duration_s: float, rate_hz: float) -> None:
    dt = 1.0 / rate_hz
    end = time.monotonic() + duration_s

    while time.monotonic() < end:
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw_deg))
        await asyncio.sleep(dt)


async def fly_figure8(args: argparse.Namespace) -> None:
    drone = System()
    await drone.connect(system_address=args.connection)

    await wait_connected(drone)
    await wait_ready(drone)

    down = -args.altitude
    rate_hz = args.rate
    dt = 1.0 / rate_hz

    print("Arming...")
    await drone.action.arm()

    print("Sending initial offboard setpoint...")
    await stream_position(drone, 0.0, 0.0, down, 0.0, 1.0, rate_hz)
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, down, 0.0))

    print("Starting offboard...")

    try:
        await drone.offboard.start()

    except OffboardError as error:
        print(f"Offboard start failed: {error._result.result}")
        await drone.action.disarm()
        raise

    print(f"Climbing to {args.altitude:.1f} m...")
    await stream_position(drone, 0.0, 0.0, down, 0.0, 2.0, rate_hz)
    reached = await wait_position_reached(drone, 0.0, 0.0, down, args.acceptance_radius, args.takeoff_timeout)
    print(f"Takeoff position reached: {reached}")

    total_time = args.period * args.laps
    omega = 2.0 * math.pi / args.period
    start = time.monotonic()
    next_print = 0.0

    print(
        f"Flying figure-8: radius={args.radius:.1f} m, period={args.period:.1f} s, "
        f"laps={args.laps:.1f}, altitude={args.altitude:.1f} m"
    )

    while True:
        elapsed = time.monotonic() - start

        if elapsed > total_time:
            break

        phase = omega * elapsed

        # Gerono lemniscate in local NED: north = R sin(t), east = R sin(t) cos(t).
        north = args.radius * math.sin(phase)
        east = args.radius * math.sin(phase) * math.cos(phase)

        # Face along approximate tangent direction.
        dn = args.radius * omega * math.cos(phase)
        de = args.radius * omega * math.cos(2.0 * phase)
        yaw_deg = math.degrees(math.atan2(de, dn))

        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw_deg))

        if elapsed >= next_print:
            print(f"  t={elapsed:6.1f}s setpoint N/E/D/yaw = {north: .2f}, {east: .2f}, {down: .2f}, {yaw_deg: .1f}")
            next_print += 5.0

        await asyncio.sleep(dt)

    print("Returning to center...")
    await stream_position(drone, 0.0, 0.0, down, 0.0, args.settle_time, rate_hz)

    print("Stopping offboard and landing...")

    try:
        await drone.offboard.stop()

    except OffboardError as error:
        print(f"Offboard stop failed: {error._result.result}")

    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break

    print("Landed")


def main() -> int:
    parser = argparse.ArgumentParser(description="Repeatable PX4 SIH/SITL offboard figure-8 maneuver")
    parser.add_argument("--connection", default="udp://:14540", help="MAVSDK connection URL")
    parser.add_argument("--altitude", type=float, default=5.0, help="Altitude above home [m]")
    parser.add_argument("--radius", type=float, default=8.0, help="Figure-8 radius scale [m]")
    parser.add_argument("--period", type=float, default=10.0, help="Time for one figure-8 cycle [s]")
    parser.add_argument("--laps", type=float, default=4.0, help="Number of figure-8 cycles")
    parser.add_argument("--rate", type=float, default=20.0, help="Offboard setpoint stream rate [Hz]")
    parser.add_argument("--acceptance-radius", type=float, default=0.8, help="Takeoff position acceptance radius [m]")
    parser.add_argument("--takeoff-timeout", type=float, default=45.0, help="Takeoff wait timeout [s]")
    parser.add_argument("--settle-time", type=float, default=5.0, help="Center hold time after maneuver [s]")
    args = parser.parse_args()

    try:
        asyncio.run(fly_figure8(args))

    except KeyboardInterrupt:
        print("Interrupted")
        return 1

    except Exception as error:
        print(f"Failed: {error}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
