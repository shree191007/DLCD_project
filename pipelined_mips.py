"""Pipelined MIPS in gem5: MipsMinorCPU = in-order, 4-stage pipeline
(Fetch1 -> Fetch2 -> Decode -> Execute) with scoreboard-based hazard handling."""
import argparse, os
import m5
from m5.objects import *

p = argparse.ArgumentParser()
p.add_argument("--binary", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "sum_pipeline.elf"))
p.add_argument("--cpu", default="minor", choices=["minor", "simple", "o3"])
a = p.parse_args()

system = System()
system.clk_domain = SrcClockDomain(clock="1GHz", voltage_domain=VoltageDomain())
system.mem_mode = "timing"
system.mem_ranges = [AddrRange("512MiB")]

system.cpu = {"minor": MipsMinorCPU, "simple": MipsTimingSimpleCPU, "o3": MipsO3CPU}[a.cpu]()
system.membus = SystemXBar()
system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports
system.cpu.createInterruptController()

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports
system.system_port = system.membus.cpu_side_ports

system.workload = SEWorkload.init_compatible(a.binary)
proc = Process(cmd=[a.binary])
system.cpu.workload = proc
system.cpu.createThreads()

root = Root(full_system=False, system=system)
m5.instantiate()
print("Beginning simulation with", type(system.cpu).__name__)
ev = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {ev.getCause()}")
