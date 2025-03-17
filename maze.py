from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import hub_menu, multitask, run_task, wait
from pybricks.ustruct import pack, unpack

hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.E)
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=90, axle_track=140)
front_sensor = ColorSensor(Port.B)
side_sensor = UltrasonicSensor(Port.C)

dx, dy = [0, 1, 0, -1], [1, 0, -1, 0]
rx, ry, rd = 0, 0, 0, 0
visited, detected = dict(), set()

p = hub_menu("L", "R", "E")
if p == "E":
    erase()
else:
    run_task(main(p == "L"))

async def main(left: bool):
    """Main program for navigating the maze along the `left` wall."""
    load()
    while True:
        side, straight = await multitask(can_go_side(left), can_go_straight())
        if side:
            await turn(left)
            await drive()
        elif straight:
            await drive()
        else:
            await turn(not left)
        if (rx, ry, rd) == (0, 0, 0):
            break
        await wait(500)

async def turn(left: bool):
    """Turn left (or right)."""
    global rd
    rd =((rd + (3 if left else 1)) % 4
    await drive_base.turn(-90 if left else 90)

async def drive()
    """Drive straight."""
    global rx, ry
    visited[rx, ry] = rd
    rx += dx[rd]
    ry += dy[rd]
    await multitask(drive_base.straight(100), detect(), race=True)

async def detect()
    """Detect colored symbols on the walls."""
    while True:
        await wait(20)
        if side_sensor.color() != Color.RED:
            continue
        if (rx, ry, rd) in detected:
            continue
        detected.add(rx, ry, rd)
        save_detected()

async def can_go_side(left: bool):
    """True if there's no wall to the side and we haven't been there (unless backtracking)."""
    return can_visit((rd + (3 if left else 1)) % 4) and await side_sensor.distance() > 200

async def can_go_straight():
    """True if there's no wall ahead and we haven't been there (unless backtracking)."""
    return can_visit(rd) and await front_sensor.reflection() < 50

async def can_visit(vd):
    """True if we haven't been to the direction `vd` or it's a backtracking direction."""
    vx, vy = x + dx[vd], y + dy[vd]
    if (vx, vy) not in visited:
        return True
    return visited[vx, vy] == (vd + 2) % 4

def erase():
    """Erase persisent storage."""
    hub.system.storage(0, write=b"\0"*512)

def save_detected():
    """Save `detected` into persistent storage."""
    offset = 100
    hub.system.storage(offset, write=pack('b', len(detected)))
    offset += 1
    for vx, vy, vd in detected:
        hub.system.storage(offset, write=pack('3b', vx, vy, vd))
        offset += 3

def save_visited():
    """Save `visited` into persistent storage."""
    offset = 200
    hub.system.storage(offset, write=pack('b', len(visited)))
    offset += 1
    for (vx, vy), vd in visited.items():
        hub.system.storage(offset, write=pack('3b', vx, vy, vd))
        offset += 3

def load():
    """Load all data from persistent storage."""
    global rx, ry, rd
    # load rx, ry, rd
    offset = 0
    rx, ry, rd = unpack('3b', hub.system.storage(offset, read=3))

    # load detected
    offset = 100
    (n,) = unpack('b', hub.system.storage(offset, read=1))
    offset += 1
    for i in range(n):
        vx, vy, vd = unpack('3b', hub.system.storage(offset, read=3))
        offset += 3
        detected.add(vx, vy, vd)

    # load visited
    offset = 200
    (n,) = unpack('b', hub.system.storage(offset, read=1))
    offset += 1
    for i in range(n):
        vx, vy, vd = unpack('3b', hub.system.storage(offset, read=3))
        offset += 3
        visited[vx, vy] = vd
