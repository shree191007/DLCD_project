#!/usr/bin/env python3
"""Tiny MIPS32 (little-endian, o32) assembler -> static Linux ELF for gem5 SE mode.
No MIPS cross-compiler needed. Program: sum 1..10 with RAW + load-use hazards."""
import struct, sys

REG = {n: i for i, n in enumerate(
    "zero at v0 v1 a0 a1 a2 a3 t0 t1 t2 t3 t4 t5 t6 t7 s0 s1 s2 s3 s4 s5 s6 s7 t8 t9 k0 k1 gp sp fp ra".split())}
r = lambda n: REG[n]
def R(rs, rt, rd, funct, sh=0): return (rs << 21) | (rt << 16) | (rd << 11) | (sh << 6) | funct
def I(op, rs, rt, imm): return (op << 26) | (rs << 21) | (rt << 16) | (imm & 0xFFFF)

prog_tail = None
LOOP = 3  # instruction index of 'loop:'
prog = [
    I(9, r("zero"), r("t0"), 0),                 #  0 addiu t0,zero,0     sum = 0
    I(9, r("zero"), r("t1"), 1),                 #  1 addiu t1,zero,1     i = 1
    I(9, r("zero"), r("t2"), 11),                #  2 addiu t2,zero,11    limit
    R(r("t0"), r("t1"), r("t0"), 0x21),          #  3 loop: addu t0,t0,t1 (RAW on t0)
    I(9, r("t1"), r("t1"), 1),                   #  4 addiu t1,t1,1
    I(5, r("t1"), r("t2"), LOOP - (5 + 1)),      #  5 bne t1,t2,loop      (data hazard on t1)
    0,                                           #  6 nop (delay slot)
    I(0x2B, r("sp"), r("t0"), -4),               #  7 sw t0,-4(sp)
    I(0x23, r("sp"), r("t3"), -4),               #  8 lw t3,-4(sp)
    R(r("t3"), r("t3"), r("t4"), 0x21),          #  9 addu t4,t3,t3       (load-use hazard)
    R(r("t3"), r("zero"), r("a0"), 0x21),        # 10 addu a0,t3,zero     exit code = 55
    I(9, r("zero"), r("v0"), 4001),              # 11 addiu v0,zero,4001  SYS_exit (o32)
    0x0000000C,                                  # 12 syscall
]
prog_tail = prog[7:]
if "--unrolled" in sys.argv:   # branch-free variant (gem5 Minor mishandles MIPS delay slots)
    sys.argv.remove("--unrolled")
    prog = [I(9, r("zero"), r("t0"), 0)]
    for i in range(1, 11):
        prog += [I(9, r("zero"), r("t1"), i), R(r("t0"), r("t1"), r("t0"), 0x21)]   # RAW on t1 and t0
    prog += prog_tail
code = b"".join(struct.pack("<I", w & 0xFFFFFFFF) for w in prog)
BASE, HDR = 0x400000, 52 + 32
ident = b"\x7fELF" + bytes([1, 1, 1, 3]) + b"\0" * 8      # 32-bit, LE, v1, OSABI=Linux
ehdr = ident + struct.pack("<HHIIIIIHHHHHH", 2, 8, 1, BASE + HDR, 52, 0, 0x1004, 52, 32, 1, 0, 0, 0)
size = HDR + len(code)
phdr = struct.pack("<IIIIIIII", 1, 0, BASE, BASE, size, size, 7, 0x1000)
open(sys.argv[1] if len(sys.argv) > 1 else "sum_pipeline.elf", "wb").write(ehdr + phdr + code)
print("wrote ELF,", len(prog), "instructions")
