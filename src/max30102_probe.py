# -*- coding: utf-8 -*-
import max30102

if __name__ == "__main__":
    m = max30102.MAX30102(gpio_pin=None)
    print("Channel: 1, address: 0x57")
    print("Reading 2s of samples...")
    red, ir = m.read_sequential(200)
    print(f"Collected: {len(red)} red, {len(ir)} IR samples")
    if ir:
        print(f"IR range: {min(ir)} → {max(ir)}")
