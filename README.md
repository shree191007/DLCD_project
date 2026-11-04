# Pipelined MIPS: Verilog RTL + gem5 simulation

DLCD project. A 5-stage pipelined MIPS processor (IF/ID/EX/MEM/WB) in Verilog with forwarding, load-use stall and
branch flush, plus a cycle-level comparison using the gem5 simulator.

## Layout
| Path | What |
|---|---|
| `verilog/pipelined_mips.v`, `verilog/pipelined_mips_tb.v` | RTL design and testbench (all 10 instructions) |
| `mips_asm.py` | tiny MIPS assembler that writes static ELF files for gem5 (no cross-compiler needed) |
| `pipelined_mips.py` | gem5 config (`--cpu minor|simple|o3`, `--binary`) |
| `sum_pipeline.elf`, `sum_unrolled.elf` | test programs (loop with delay slot; branch-free) |
| `outputs/minor_delay_slot_fix.patch` | patch to gem5's Minor CPU for MIPS branch delay slots |
| `outputs/` | simulation outputs, waveform, gem5 stats, final report (`.docx`, `.md`) |
| `report.md` | project report |

## Run the Verilog simulation (Icarus Verilog)
```
cd verilog
iverilog -g2012 -o pm.vvp pipelined_mips.v pipelined_mips_tb.v && vvp pm.vvp   # prints RESULT: PASS
gtkwave pipelined_mips.vcd
```

## Run gem5
```
git clone https://github.com/gem5/gem5.git && cd gem5
git apply ../outputs/minor_delay_slot_fix.patch
printf 'BUILD_ISA=y\nUSE_MIPS_ISA=y\n' > build_opts/MIPS_LITE
scons build/MIPS_LITE/gem5.opt -j8
build/MIPS_LITE/gem5.opt ../pipelined_mips.py --cpu minor --binary ../sum_unrolled.elf
python3 ../mips_asm.py ../sum_unrolled.elf --unrolled   # regenerate the ELFs if needed
```
Results: see `outputs/gem5_summary.txt` and section 5 of `report.md`.
