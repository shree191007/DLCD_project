`timescale 1ns/1ps
module pipelined_mips_tb;
  reg clk = 0, reset = 1;
  pipelined_mips dut(.clk(clk), .reset(reset));
  always #5 clk = ~clk;

  function [31:0] R(input [5:0] fn, input [4:0] rs, rt, rd);  R = {6'd0, rs, rt, rd, 5'd0, fn}; endfunction
  function [31:0] I(input [5:0] op, input [4:0] rs, rt, input [15:0] imm); I = {op, rs, rt, imm}; endfunction

  integer k;
  initial begin
    $dumpfile("pipelined_mips.vcd"); $dumpvars(0, pipelined_mips_tb);
    for (k = 0; k < 64; k = k + 1) begin dut.imem[k] = 0; dut.dmem[k] = 0; end
    for (k = 0; k < 32; k = k + 1) dut.rf[k] = 0;
    // sum = 1+2+...+10 ; store/load ; load-use hazard
    dut.imem[0]  = I(6'h08, 0, 8, 0);            // addi $t0,$0,0
    dut.imem[1]  = I(6'h08, 0, 9, 1);            // addi $t1,$0,1
    dut.imem[2]  = I(6'h08, 0, 10, 11);          // addi $t2,$0,11
    dut.imem[3]  = R(6'h20, 8, 9, 8);            // loop: add $t0,$t0,$t1   (RAW -> forwarding)
    dut.imem[4]  = I(6'h08, 9, 9, 1);            // addi $t1,$t1,1
    dut.imem[5]  = I(6'h05, 9, 10, 16'hFFFD);    // bne $t1,$t2,loop        (-3)
    dut.imem[6]  = I(6'h2B, 0, 8, 0);            // sw $t0,0($0)
    dut.imem[7]  = I(6'h23, 0, 11, 0);           // lw $t3,0($0)
    dut.imem[8]  = R(6'h20, 11, 11, 12);         // add $t4,$t3,$t3         (load-use stall)
    dut.imem[9]  = R(6'h22, 12, 8, 13);          // sub $t5,$t4,$t0 = 110-55 (forward from EX/MEM)
    dut.imem[10] = R(6'h24, 13, 11, 14);         // and $t6,$t5,$t3 = 55&55
    dut.imem[11] = R(6'h25, 8, 12, 15);          // or  $t7,$t0,$t4 = 55|110
    dut.imem[12] = R(6'h2A, 8, 12, 16);          // slt $s0,$t0,$t4 = 1
    dut.imem[13] = I(6'h04, 14, 13, 16'h0001);   // beq $t6,$t5,+1  (taken: skips next)
    dut.imem[14] = I(6'h08, 0, 17, 99);          // addi $s1,$0,99  (must be skipped)
    dut.imem[15] = I(6'h08, 0, 18, 7);           // addi $s2,$0,7
    dut.imem[16] = I(6'h04, 0, 0, 16'hFFFF);     // beq $0,$0,-1 (halt: branch to self)
    #22 reset = 0;
    repeat (80) @(posedge clk);
    #1;
    $display("---- Register / memory results ----");
    $display("t0 (sum 1..10)   = %0d (expect 55)",  dut.rf[8]);
    $display("t1 (i)           = %0d (expect 11)",  dut.rf[9]);
    $display("t3 (lw result)   = %0d (expect 55)",  dut.rf[11]);
    $display("t4 (t3+t3)       = %0d (expect 110)", dut.rf[12]);
    $display("dmem[0]          = %0d (expect 55)",  dut.dmem[0]);
    $display("t5 (sub)         = %0d (expect 55)",  dut.rf[13]);
    $display("t6 (and)         = %0d (expect 55)",  dut.rf[14]);
    $display("t7 (or)          = %0d (expect 127)", dut.rf[15]);
    $display("s0 (slt)         = %0d (expect 1)",   dut.rf[16]);
    $display("s1 (skipped)     = %0d (expect 0, beq taken)", dut.rf[17]);
    $display("s2               = %0d (expect 7)",   dut.rf[18]);
    $display("---- Pipeline events (80 cycles incl. halt spin) ----");
    $display("load-use stalls = %0d, branch-flushed slots = %0d", dut.stalls, dut.flushes);
    if (dut.rf[8]==55 && dut.rf[9]==11 && dut.rf[11]==55 && dut.rf[12]==110 && dut.dmem[0]==55 &&
        dut.rf[13]==55 && dut.rf[14]==55 && dut.rf[15]==127 && dut.rf[16]==1 && dut.rf[17]==0 && dut.rf[18]==7)
      $display("RESULT: PASS"); else $display("RESULT: FAIL");
    $finish;
  end
endmodule
