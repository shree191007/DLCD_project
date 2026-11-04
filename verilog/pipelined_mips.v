`timescale 1ns/1ps
// 5-stage pipelined MIPS (IF, ID, EX, MEM, WB)
// Supports: add sub and or slt addi lw sw beq bne
// Hazards : full forwarding (EX/MEM, MEM/WB -> EX), load-use stall (1 cycle),
//           branches resolved in EX (taken branch flushes 2 instructions)
module pipelined_mips(input clk, input reset);
  // ---------------- state ----------------
  reg [31:0] pc;
  reg [31:0] imem [0:63];
  reg [31:0] dmem [0:63];
  reg [31:0] rf   [0:31];

  // pipeline registers
  reg [31:0] ifid_instr, ifid_pc4;
  // ID/EX
  reg [31:0] idex_a, idex_b, idex_imm, idex_pc4;
  reg [4:0]  idex_rs, idex_rt, idex_rd;
  reg        idex_regwrite, idex_memtoreg, idex_memread, idex_memwrite;
  reg        idex_alusrc, idex_regdst, idex_beq, idex_bne;
  reg [2:0]  idex_aluop;
  // EX/MEM
  reg [31:0] exmem_alu, exmem_wdata;
  reg [4:0]  exmem_wr;
  reg        exmem_regwrite, exmem_memtoreg, exmem_memread, exmem_memwrite;
  // MEM/WB
  reg [31:0] memwb_alu, memwb_rdata;
  reg [4:0]  memwb_wr;
  reg        memwb_regwrite, memwb_memtoreg;

  // performance counters
  integer cycles, stalls, flushes, retired;

  // ---------------- ID stage ----------------
  wire [5:0] opcode = ifid_instr[31:26];
  wire [4:0] rs = ifid_instr[25:21], rt = ifid_instr[20:16], rd = ifid_instr[15:11];
  wire [5:0] funct = ifid_instr[5:0];
  wire [31:0] imm_se = {{16{ifid_instr[15]}}, ifid_instr[15:0]};

  reg c_regwrite, c_memtoreg, c_memread, c_memwrite, c_alusrc, c_regdst, c_beq, c_bne;
  reg [2:0] c_aluop;   // 0 add,1 sub,2 and,3 or,4 slt
  always @* begin
    {c_regwrite,c_memtoreg,c_memread,c_memwrite,c_alusrc,c_regdst,c_beq,c_bne} = 8'b0;
    c_aluop = 3'd0;
    case (opcode)
      6'h00: if (ifid_instr != 0) begin
               c_regwrite = 1; c_regdst = 1;
               case (funct)
                 6'h20: c_aluop = 0; 6'h22: c_aluop = 1; 6'h24: c_aluop = 2;
                 6'h25: c_aluop = 3; 6'h2A: c_aluop = 4;
                 default: c_regwrite = 0;
               endcase
             end
      6'h08: begin c_regwrite = 1; c_alusrc = 1; end              // addi
      6'h23: begin c_regwrite = 1; c_alusrc = 1; c_memread = 1; c_memtoreg = 1; end // lw
      6'h2B: begin c_alusrc = 1; c_memwrite = 1; end              // sw
      6'h04: begin c_beq = 1; c_aluop = 1; end                    // beq
      6'h05: begin c_bne = 1; c_aluop = 1; end                    // bne
    endcase
  end

  // register read with WB->ID bypass (write in first half of cycle)
  wire [31:0] rd1 = (rs == 0) ? 0 : (memwb_regwrite && memwb_wr == rs) ? (memwb_memtoreg ? memwb_rdata : memwb_alu) : rf[rs];
  wire [31:0] rd2 = (rt == 0) ? 0 : (memwb_regwrite && memwb_wr == rt) ? (memwb_memtoreg ? memwb_rdata : memwb_alu) : rf[rt];

  // load-use hazard detection
  wire stall = idex_memread && (idex_rt != 0) && (idex_rt == rs || idex_rt == rt);

  // ---------------- EX stage ----------------
  wire [1:0] fwdA = (exmem_regwrite && exmem_wr != 0 && exmem_wr == idex_rs) ? 2'd1 :
                    (memwb_regwrite && memwb_wr != 0 && memwb_wr == idex_rs) ? 2'd2 : 2'd0;
  wire [1:0] fwdB = (exmem_regwrite && exmem_wr != 0 && exmem_wr == idex_rt) ? 2'd1 :
                    (memwb_regwrite && memwb_wr != 0 && memwb_wr == idex_rt) ? 2'd2 : 2'd0;
  wire [31:0] wb_val = memwb_memtoreg ? memwb_rdata : memwb_alu;
  wire [31:0] opA = fwdA == 1 ? exmem_alu : fwdA == 2 ? wb_val : idex_a;
  wire [31:0] opB_pre = fwdB == 1 ? exmem_alu : fwdB == 2 ? wb_val : idex_b;
  wire [31:0] opB = idex_alusrc ? idex_imm : opB_pre;

  reg [31:0] alu_out;
  always @* case (idex_aluop)
    0: alu_out = opA + opB;
    1: alu_out = opA - opB;
    2: alu_out = opA & opB;
    3: alu_out = opA | opB;
    4: alu_out = ($signed(opA) < $signed(opB)) ? 32'd1 : 32'd0;
    default: alu_out = 0;
  endcase
  wire zero = (alu_out == 0);
  wire take_branch = (idex_beq && zero) || (idex_bne && !zero);
  wire [31:0] branch_target = idex_pc4 + (idex_imm << 2);

  // ---------------- MEM stage ----------------
  wire [31:0] mem_rdata = dmem[exmem_alu[7:2]];

  // ---------------- sequential logic ----------------
  integer i;
  always @(posedge clk) begin
    if (reset) begin
      pc <= 0; ifid_instr <= 0; ifid_pc4 <= 0;
      idex_regwrite <= 0; idex_memread <= 0; idex_memwrite <= 0; idex_beq <= 0; idex_bne <= 0;
      idex_memtoreg <= 0; idex_rt <= 0; idex_rs <= 0; idex_rd <= 0; idex_aluop <= 0;
      idex_alusrc <= 0; idex_regdst <= 0; idex_a <= 0; idex_b <= 0; idex_imm <= 0; idex_pc4 <= 0;
      exmem_regwrite <= 0; exmem_memwrite <= 0; exmem_memread <= 0; exmem_memtoreg <= 0;
      exmem_alu <= 0; exmem_wdata <= 0; exmem_wr <= 0;
      memwb_regwrite <= 0; memwb_memtoreg <= 0; memwb_alu <= 0; memwb_rdata <= 0; memwb_wr <= 0;
      cycles <= 0; stalls <= 0; flushes <= 0; retired <= 0;
    end else begin
      cycles <= cycles + 1;
      // WB
      if (memwb_regwrite && memwb_wr != 0) rf[memwb_wr] <= wb_val;
      if (memwb_regwrite || memwb_wr != 0) ; // (retire counting below)
      // MEM
      if (exmem_memwrite) dmem[exmem_alu[7:2]] <= exmem_wdata;
      memwb_regwrite <= exmem_regwrite; memwb_memtoreg <= exmem_memtoreg;
      memwb_alu <= exmem_alu; memwb_rdata <= mem_rdata; memwb_wr <= exmem_wr;
      // EX
      exmem_regwrite <= idex_regwrite; exmem_memtoreg <= idex_memtoreg;
      exmem_memread <= idex_memread;  exmem_memwrite <= idex_memwrite;
      exmem_alu <= alu_out; exmem_wdata <= opB_pre;
      exmem_wr <= idex_regdst ? idex_rd : idex_rt;
      // ID / IF with hazard control
      if (take_branch) begin
        flushes <= flushes + 2;
        pc <= branch_target;
        ifid_instr <= 0; ifid_pc4 <= 0;
        idex_regwrite <= 0; idex_memread <= 0; idex_memwrite <= 0; idex_beq <= 0; idex_bne <= 0; idex_memtoreg <= 0;
        idex_rt <= 0; idex_rs <= 0; idex_rd <= 0;
      end else if (stall) begin
        stalls <= stalls + 1;                       // hold PC and IF/ID, insert bubble
        idex_regwrite <= 0; idex_memread <= 0; idex_memwrite <= 0; idex_beq <= 0; idex_bne <= 0; idex_memtoreg <= 0;
        idex_rt <= 0; idex_rs <= 0; idex_rd <= 0;
      end else begin
        pc <= pc + 4;
        ifid_instr <= imem[pc[7:2]]; ifid_pc4 <= pc + 4;
        idex_a <= rd1; idex_b <= rd2; idex_imm <= imm_se; idex_pc4 <= ifid_pc4;
        idex_rs <= rs; idex_rt <= rt; idex_rd <= rd;
        idex_regwrite <= c_regwrite; idex_memtoreg <= c_memtoreg; idex_memread <= c_memread;
        idex_memwrite <= c_memwrite; idex_alusrc <= c_alusrc; idex_regdst <= c_regdst;
        idex_beq <= c_beq; idex_bne <= c_bne; idex_aluop <= c_aluop;
      end
    end
  end
endmodule
