# IT10 co-simulation, Renode side (IronPython 2.7, run inside Renode 1.17).
#
# Loaded after the weapon firmware's sim/lane.resc (two trainer_fw machines, node 1 and node 2,
# on one CAN hub). Drives the lane the way the lane controller and two
# shooters would, and records what the firmware does:
#
#   - SYNC every 10 ms (ISO-TP single frame, lane time = virtual time + LANE0_US)
#   - camera mode on both weapons: STROBE_SLOT (key 14) = 0 / 1, STROBE_SLOTS (key 13) = 2
#   - SAFE -> SEMI, magazine in, rack (buttons sw1, sw2, sw3)
#   - trigger pulls at fixed times on alternating weapons
#
# Recorded: every emitter gate edge (LED StateChanged, machine virtual time)
# and every CAN frame each weapon sends (MCAN FrameSent), plus the frames
# injected here. Written as plain text lines to $COSIM_OUT:
#
#   E <node> <vt_us> <0|1>         emitter edge
#   T <node> <vt_us> <id> <hex>    frame sent by the weapon
#   I <vt_us> <id> <hex>           frame injected by the lane (SYNC, CMD)
#   L <lane0_us>                   lane time = virtual time + lane0
import clr
from System import Array, Byte, Environment
from System.IO import Path
from Antmicro.Renode.Core import EmulationManager
from Antmicro.Renode.Peripherals import IPeripheral
from Antmicro.Renode.Time import TimeInterval
import Antmicro.Renode.Core.CAN as CC

OUT = Environment.GetEnvironmentVariable("COSIM_OUT") or Path.Combine(Path.GetTempPath(), "cosim_events.txt")
LANE0_US = 5000000
STEP_US = 10000          # SYNC period, and the driver's time step
END_US = 3400000
SHOTS = {1: [1000000, 1500000, 2000000, 2500000], 2: [1250000, 1750000, 2250000, 2750000]}
HOLD_US = 100000         # trigger held this long

emu = EmulationManager.Instance.CurrentEmulation
print("cosim: quantum %s" % emu.MasterTimeSource.Quantum)
lines = []


def periph(m, name):
    ref = clr.Reference[IPeripheral]()
    if not m.TryGetByName[IPeripheral](name, ref):
        raise Exception("no peripheral " + name)
    return ref.Value


def vt_us(m):
    return int(m.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds)


machines = {}
for m in emu.Machines:
    node = 1 if emu[m] == "weapon1" else 2
    machines[node] = m


def hook(node, m):
    led = periph(m, "sysbus.gpioPortE.emitterGate")
    can = periph(m, "sysbus.fdcan1")

    def on_led(_led, state):
        lines.append("E %d %d %d" % (node, vt_us(m), 1 if state else 0))

    def on_frame(f):
        lines.append("T %d %d %X %s" % (node, vt_us(m), f.Id, "".join("%02X" % b for b in f.Data)))

    led.StateChanged += on_led
    can.FrameSent += on_frame


for node in machines:
    hook(node, machines[node])


def inject(fid, data):
    t = vt_us(machines[1])
    lines.append("I %d %X %s" % (t, fid, "".join("%02X" % b for b in data)))
    frame = CC.CANMessageFrame(fid, Array[Byte](data), False, False, False, False)
    for node in machines:
        periph(machines[node], "sysbus.fdcan1").OnFrameReceived(frame)


def u32(v):
    v &= 0xFFFFFFFF
    return [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]


def cmd(node, key, value):
    inject((7 << 7) | node, [6, 2, key] + u32(value))


def button(node, sw, pressed):
    b = periph(machines[node], "sysbus.gpioPortG." + sw)
    if pressed:
        b.Press()
    else:
        b.Release()


def trigger(node, percent):
    periph(machines[node], "sysbus.adcM1S2.potentiometer").SetPercentage(percent)


# Actions on the 10 ms grid: time (us) -> list of callables.
actions = {}


def at(t, fn):
    actions.setdefault(t - t % STEP_US, []).append(fn)


at(300000, lambda: cmd(1, 14, 0))
at(310000, lambda: cmd(2, 14, 1))
at(320000, lambda: cmd(1, 13, 2))
at(330000, lambda: cmd(2, 13, 2))
for i, sw in enumerate(("sw1", "sw2", "sw3")):
    for node in (1, 2):
        at(400000 + i * 100000, lambda n=node, s=sw: button(n, s, True))
        at(450000 + i * 100000, lambda n=node, s=sw: button(n, s, False))
for node in SHOTS:
    for t in SHOTS[node]:
        at(t, lambda n=node: trigger(n, 100))
        at(t + HOLD_US, lambda n=node: trigger(n, 0))

lines.append("L %d" % LANE0_US)
t = 0
while t < END_US:
    inject(1 << 7, [4] + u32(t + LANE0_US))  # SYNC carries the lane time at this instant
    for fn in actions.get(t, []):
        fn()
    emu.RunFor(TimeInterval.FromMicroseconds(STEP_US))
    t += STEP_US

f = open(OUT, "w")
f.write("\n".join(lines) + "\n")
f.close()
print("cosim: %d lines -> %s" % (len(lines), OUT))
