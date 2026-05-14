import argparse
import can
import time

def parse_nibble(s):
    s = str(s).strip().lower()
    if s.startswith("0x"):
        return int(s, 16) & 0xF
    if set(s) <= {"0", "1"}:
        return int(s, 2) & 0xF
    return int(s) & 0xF

def build_bytes(g1, g2, g3):
    b0 = (g1 & 0xF) | ((g2 & 0xF) << 4)
    b1 = g3 & 0xF
    return b0, b1

def send_write(bus, address, g1, g2, g3):
    b0, b1 = build_bytes(g1, g2, g3)
    data = [b0, b1, 0, 0, 0, 0, 0, 0]
    msg = can.Message(arbitration_id=(0x01 << 8) | (address & 0xFF), data=data, is_extended_id=False)
    print(f"TX ID=0x{msg.arbitration_id:03X} DATA={bytes(data).hex()}")
    bus.send(msg)

def send_read(bus, address):
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    msg = can.Message(arbitration_id=(0x02 << 8) | (address & 0xFF), data=data, is_extended_id=False)
    print(f"TX ID=0x{msg.arbitration_id:03X} DATA={bytes(data).hex()}")
    bus.send(msg)

def channels_to_groups(ch_str):
    ch = []
    for part in str(ch_str).split(","):
        part = part.strip()
        if not part:
            continue
        ch.append(int(part, 0))
    g1 = 0
    g2 = 0
    g3 = 0
    for c in ch:
        if 1 <= c <= 4:
            g1 |= 1 << (c - 1)
        elif 5 <= c <= 8:
            g2 |= 1 << (c - 5)
        elif 9 <= c <= 12:
            g3 |= 1 << (c - 9)
    return g1 & 0xF, g2 & 0xF, g3 & 0xF

def wait_reply(bus, timeout=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = bus.recv(0.2)
        if m is not None:
            print(f"RX ID=0x{m.arbitration_id:03X} DATA={m.data.hex()}")
            return m
    print("RX timeout")
    return None

def show_read_bits(msg):
    func = (msg.arbitration_id >> 8) & 0x07
    if func != 0x02:
        return
    b0 = msg.data[0] if len(msg.data) > 0 else 0
    b1 = msg.data[1] if len(msg.data) > 1 else 0
    g1 = b0 & 0x0F
    g2 = (b0 >> 4) & 0x0F
    g3 = b1 & 0x0F
    print(f"G1(1-4)={g1:04b} G2(5-8)={g2:04b} G3(9-12)={g3:04b}")
    on_ch = []
    for i in range(4):
        if g1 & (1 << i):
            on_ch.append(1 + i)
    for i in range(4):
        if g2 & (1 << i):
            on_ch.append(5 + i)
    for i in range(4):
        if g3 & (1 << i):
            on_ch.append(9 + i)
    print(f"ON channels={on_ch}")

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interface", default="virtual")
    p.add_argument("--channel", default="vcan0")
    p.add_argument("--bitrate", type=int, default=115200)
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x01)
    p.add_argument("--g1", default=None)
    p.add_argument("--g2", default=None)
    p.add_argument("--g3", default=None)
    p.add_argument("--channels", default=None)
    p.add_argument("--read", action="store_true")
    args = p.parse_args()

    write_requested = bool(args.channels or args.g1 or args.g2 or args.g3)

    bus = can.interface.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    if write_requested:
        if args.channels:
            g1, g2, g3 = channels_to_groups(args.channels)
        else:
            g1 = parse_nibble(args.g1 or "0")
            g2 = parse_nibble(args.g2 or "0")
            g3 = parse_nibble(args.g3 or "0")
        send_write(bus, args.address, g1, g2, g3)
        wait_reply(bus, 1.0)
        if args.read:
            send_read(bus, args.address)
            m = wait_reply(bus, 1.0)
            if m:
                show_read_bits(m)
    else:
        send_read(bus, args.address)
        m = wait_reply(bus, 1.0)
        if m:
            show_read_bits(m)
    bus.shutdown()

if __name__ == "__main__":
    main()
