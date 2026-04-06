module riscv (
    input  logic        clk, reset,
    input  logic [31:0] ReadDataM,
    input  logic [31:0] InstrF,    // instruction at PCF
    input  logic [31:0] InstrF2,   // instruction at PCF+4 (for dual fetch)
    output logic [31:0] PCF,       // first fetch address
    output logic [31:0] PCF2,      // second fetch address (= PCF+4)
    output logic        MemWriteM,
    output logic [31:0] DataAdrM, WriteDataM 
);

    // Control signals between controller, datapath, hazard unit
    logic        RegWrite, RegWriteM, RegWriteW;
    logic        MemWriteE;
    logic        ALUSrcE;
    logic [2:0]  ImmSrcD;
    logic [2:0]  ALUControlE;
    logic [1:0]  ResultSrcE, ResultSrcW;

    // Hazard unit signals
    logic        StallF, StallD;
    logic        FlushD, FlushE;
    logic [1:0]  ForwardAE, ForwardBE;

    // Datapath outputs
    logic        ZeroE, Negative, Overflow;
    logic [6:0]  op;
    logic [2:0]  funct3;
    logic        funct7b5;
    logic [31:0] InstrD;
    logic [31:0] ResultW, ALUResultM;

    // Register identifiers for hazard detection
    logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;

    // Branch/jump signals
    logic        BranchD, JumpD;
    logic        PCSrcE;
    
    // Instruction Queue signals
    logic [31:0] instr_queue0, instr_queue1;
    logic [1:0]  queue_valid_cnt;
    logic [1:0]  dispatch_shift_amt;
    logic [1:0]  queue_space;    // free slots reported by queue
    logic [1:0]  fetch_amt;      // how many instructions to fetch this cycle
    assign fetch_amt = queue_space;
    
    // Dispatch unit signals
    logic [6:0]  disp_op0, disp_op1;
    logic [2:0]  disp_funct3_0, disp_funct3_1;
    logic        disp_funct7b5_0, disp_funct7b5_1;
    logic [4:0]  disp_rs1_0, disp_rs2_0, disp_rd_0;
    logic [4:0]  disp_rs1_1, disp_rs2_1, disp_rd_1;
    logic [31:0] disp_imm_0, disp_imm_1;
    logic [2:0]  disp_alucontrol_0, disp_alucontrol_1;
    logic [1:0]  disp_aluop_0, disp_aluop_1;
    logic        disp_regwrite_0, disp_regwrite_1;
    logic        disp_memwrite_0, disp_memwrite_1;
    logic        disp_memsrc_0, disp_memsrc_1;
    logic        disp_valid_0, disp_valid_1;
    logic [1:0]  disp_fu_type_0, disp_fu_type_1;
    logic [1:0]  disp_cnt;

    // Dispatch-time register file read data (from expanded regfile)
    logic [31:0] disp_rdata_rs1_0, disp_rdata_rs2_0; // slot 0 operands
    logic [31:0] disp_rdata_rs1_1, disp_rdata_rs2_1; // slot 1 operands

    // Tomasulo dispatch stall signals (driven by RS full/alloc flags)
    logic        rs_stall0, rs_stall1;

    // RS status
    logic        adder_rs_full, mul_rs_full, mem_rs_full;
    logic        adder_rs_alloc1_ok, mul_rs_alloc1_ok, mem_rs_alloc1_ok;

    // Allocated RS tags per dispatch slot (from RS instances, muxed to RAT)
    logic [3:0]  adder_rs_disp0_tag, adder_rs_disp1_tag;
    logic [3:0]  mul_rs_disp0_tag,   mul_rs_disp1_tag;
    logic [3:0]  mem_rs_disp0_tag,   mem_rs_disp1_tag;

    // RAT tag lookups at dispatch time
    logic [3:0]  rat_tag_rs1_0, rat_tag_rs2_0;
    logic [3:0]  rat_tag_rs1_1, rat_tag_rs2_1;
    logic [3:0]  rat_out [31:0];

    // CDB bus
    logic        cdb_valid;
    logic [3:0]  cdb_tag;
    logic [31:0] cdb_value;
    logic [4:0]  cdb_rd;

    // ADDER RS issue outputs
    logic        adder_issue_valid;
    logic [31:0] adder_issue_Vj, adder_issue_Vk, adder_issue_imm;
    logic [2:0]  adder_issue_ctrl;
    logic [3:0]  adder_issue_tag;
    logic [4:0]  adder_issue_rd;
    logic        adder_issue_memsrc;

    // MUL RS issue outputs
    logic        mul_issue_valid;
    logic [31:0] mul_issue_Vj, mul_issue_Vk;
    logic [3:0]  mul_issue_tag;
    logic [4:0]  mul_issue_rd;

    // MEM RS issue outputs
    logic        mem_issue_valid;
    logic [31:0] mem_issue_Vj, mem_issue_Vk, mem_issue_imm;
    logic        mem_issue_memwrite;
    logic [3:0]  mem_issue_tag;
    logic [4:0]  mem_issue_rd;

    // Decouple in-order controller/datapath signals from OoO output ports
    logic        dp_MemWriteM;      // controller → datapath (in-order path)
    logic [31:0] dp_WriteDataM;     // datapath store data (in-order path, unused by OoO)

    // OoO FU result signals
    logic [31:0] ooo_adder_result;
    logic [63:0] ooo_mul_result;

    // Raw FU completions (from tag pipelines, to CDB arbiter)
    logic        raw_adder_done, raw_mul_done, raw_mem_done;
    logic [3:0]  raw_adder_tag, raw_mul_tag, raw_mem_tag;
    logic [4:0]  raw_adder_rd,  raw_mul_rd,  raw_mem_rd;

    // OoO regfile write port (driven by CDB arbiter)
    logic        ooo_reg_we;
    logic [4:0]  ooo_reg_a3;
    logic [31:0] ooo_reg_wd3;

    // ------------------------
    // Instantiations
    // ------------------------

    // Instruction Queue (dual-fetch: pushes 0-2 instructions per cycle)
    instruction_queue #(.DEPTH(8), .WIDTH(32)) instr_q(
        .clk(clk),
        .reset(reset),
        .instr_in0(InstrF),
        .instr_in1(InstrF2),
        .fetch_amt(fetch_amt),
        .shift_amt(dispatch_shift_amt),
        .instr_out0(instr_queue0),
        .instr_out1(instr_queue1),
        .num_valid(queue_valid_cnt),
        .space_avail(queue_space)
    );
    
    // Dispatch Unit (decodes up to 2 instructions, gated by scoreboard stalls)
    dispatch_unit disp(
        .instr0(instr_queue0),
        .instr1(instr_queue1),
        .num_valid(queue_valid_cnt),
        .stall0_in(rs_stall0),
        .stall1_in(rs_stall1),
        .op0(disp_op0),
        .funct3_0(disp_funct3_0),
        .funct7b5_0(disp_funct7b5_0),
        .rs1_0(disp_rs1_0), .rs2_0(disp_rs2_0), .rd_0(disp_rd_0),
        .imm_0(disp_imm_0),
        .alucontrol_0(disp_alucontrol_0),
        .aluop_0(disp_aluop_0),
        .regwrite_0(disp_regwrite_0),
        .memwrite_0(disp_memwrite_0),
        .memsrc_0(disp_memsrc_0),
        .fu_type_0(disp_fu_type_0),
        .valid_0(disp_valid_0),
        .op1(disp_op1),
        .funct3_1(disp_funct3_1),
        .funct7b5_1(disp_funct7b5_1),
        .rs1_1(disp_rs1_1), .rs2_1(disp_rs2_1), .rd_1(disp_rd_1),
        .imm_1(disp_imm_1),
        .alucontrol_1(disp_alucontrol_1),
        .aluop_1(disp_aluop_1),
        .regwrite_1(disp_regwrite_1),
        .memwrite_1(disp_memwrite_1),
        .memsrc_1(disp_memsrc_1),
        .fu_type_1(disp_fu_type_1),
        .valid_1(disp_valid_1),
        .dispatch_cnt(disp_cnt)
    );

    // ================================================================
    // FU type encodings and latency constants
    // ================================================================
    localparam logic [1:0] FU_NONE_L  = 2'b00;
    localparam logic [1:0] FU_ADDER_L = 2'b01;
    localparam logic [1:0] FU_MUL_L   = 2'b10;
    localparam logic [1:0] FU_MEM_L   = 2'b11;

    localparam int ADDER_LAT = 4;
    localparam int MUL_LAT   = 6;
    localparam int MEM_LAT   = 4;  // 2-stage AGU + 1 SRAM addr register + 1 SRAM read

    // Mux to select which RS supplied each slot's allocated tag (sent to RAT)
    wire [3:0] rat_disp0_alloc_tag =
        (disp_fu_type_0 == FU_ADDER_L) ? adder_rs_disp0_tag :
        (disp_fu_type_0 == FU_MUL_L)   ? mul_rs_disp0_tag   :
        (disp_fu_type_0 == FU_MEM_L)   ? mem_rs_disp0_tag   : 4'd0;

    wire [3:0] rat_disp1_alloc_tag =
        (disp_fu_type_1 == FU_ADDER_L) ? adder_rs_disp1_tag :
        (disp_fu_type_1 == FU_MUL_L)   ? mul_rs_disp1_tag   :
        (disp_fu_type_1 == FU_MEM_L)   ? mem_rs_disp1_tag   : 4'd0;

    // ================================================================
    // Dispatch stall computation
    // Slot 0 stalls if its target RS is full OR source operands are
    // still pending in the RAT (RAW hazard at dispatch).
    // Slot 1 stalls if slot 0 stalls (in-order), or its own RS
    // cannot accept, or its own sources are pending.
    // ================================================================
    wire src_stall0 = (rat_tag_rs1_0 != 4'd0) || (rat_tag_rs2_0 != 4'd0);

    // For slot 1, allow dispatch when the pending tag is exactly the one
    // being allocated by slot 0 this cycle (intra-pair RAW).  The RS will
    // hold the tag in Qj/Qk and snoop the CDB when slot 0 completes.
    wire intra_pair_rs1_1 = disp_valid_0 && (rat_tag_rs1_1 != 4'd0)
                            && (rat_tag_rs1_1 == rat_disp0_alloc_tag);
    wire intra_pair_rs2_1 = disp_valid_0 && (rat_tag_rs2_1 != 4'd0)
                            && (rat_tag_rs2_1 == rat_disp0_alloc_tag);
    wire src_stall1 = ((rat_tag_rs1_1 != 4'd0) && !intra_pair_rs1_1)
                    || ((rat_tag_rs2_1 != 4'd0) && !intra_pair_rs2_1);

    always_comb begin
        case (disp_fu_type_0)
            FU_ADDER_L: rs_stall0 = adder_rs_full || src_stall0;
            FU_MUL_L:   rs_stall0 = mul_rs_full   || src_stall0;
            FU_MEM_L:   rs_stall0 = mem_rs_full   || src_stall0;
            default:    rs_stall0 = 1'b0;
        endcase
    end

    always_comb begin
        case (disp_fu_type_1)
            FU_ADDER_L: rs_stall1 = rs_stall0 || !adder_rs_alloc1_ok || src_stall1;
            FU_MUL_L:   rs_stall1 = rs_stall0 || !mul_rs_alloc1_ok   || src_stall1;
            FU_MEM_L:   rs_stall1 = rs_stall0 || !mem_rs_alloc1_ok   || src_stall1;
            default:    rs_stall1 = rs_stall0;
        endcase
    end

    // ================================================================
    // Register Alias Table
    // ================================================================
    rat u_rat (
        .clk          (clk),
        .reset        (reset),
        .disp0_en     (disp_valid_0 && disp_regwrite_0),
        .disp0_rd     (disp_rd_0),
        .disp0_tag    (rat_disp0_alloc_tag),
        .disp0_rs1    (disp_rs1_0),
        .disp0_rs2    (disp_rs2_0),
        .disp0_tag_rs1(rat_tag_rs1_0),
        .disp0_tag_rs2(rat_tag_rs2_0),
        .disp1_en     (disp_valid_1 && disp_regwrite_1),
        .disp1_rd     (disp_rd_1),
        .disp1_tag    (rat_disp1_alloc_tag),
        .disp1_rs1    (disp_rs1_1),
        .disp1_rs2    (disp_rs2_1),
        .disp1_tag_rs1(rat_tag_rs1_1),
        .disp1_tag_rs2(rat_tag_rs2_1),
        .cdb_valid    (cdb_valid),
        .cdb_tag      (cdb_tag),
        .rat_out      (rat_out)
    );

    // ================================================================
    // Reservation Stations  (one per FU, 3 entries each)
    //   ADDER: TAG_BASE=1 -> entry tags 1,2,3
    //   MUL:   TAG_BASE=4 -> entry tags 4,5,6
    //   MEM:   TAG_BASE=7 -> entry tags 7,8,9
    // ================================================================
    reservation_station #(.TAG_BASE(1), .DEPTH(3)) adder_rs (
        .clk            (clk),
        .reset          (reset),
        .disp0_valid    (disp_valid_0 && (disp_fu_type_0 == FU_ADDER_L)),
        .disp0_rd       (disp_rd_0),
        .disp0_alucontrol(disp_alucontrol_0),
        .disp0_memsrc   (disp_memsrc_0),
        .disp0_imm      (disp_imm_0),
        .disp0_memwrite (1'b0),
        .disp0_val_rs1  (disp_rdata_rs1_0),
        .disp0_val_rs2  (disp_rdata_rs2_0),
        .disp0_tag_rs1  (rat_tag_rs1_0),
        .disp0_tag_rs2  (rat_tag_rs2_0),
        .disp1_valid    (disp_valid_1 && (disp_fu_type_1 == FU_ADDER_L)),
        .disp1_rd       (disp_rd_1),
        .disp1_alucontrol(disp_alucontrol_1),
        .disp1_memsrc   (disp_memsrc_1),
        .disp1_imm      (disp_imm_1),
        .disp1_memwrite (1'b0),
        .disp1_val_rs1  (disp_rdata_rs1_1),
        .disp1_val_rs2  (disp_rdata_rs2_1),
        .disp1_tag_rs1  (rat_tag_rs1_1),
        .disp1_tag_rs2  (rat_tag_rs2_1),
        .cdb_valid      (cdb_valid),
        .cdb_tag        (cdb_tag),
        .cdb_value      (cdb_value),
        .issue_valid    (adder_issue_valid),
        .issue_Vj       (adder_issue_Vj),
        .issue_Vk       (adder_issue_Vk),
        .issue_ctrl     (adder_issue_ctrl),
        .issue_tag      (adder_issue_tag),
        .issue_rd       (adder_issue_rd),
        .issue_imm      (adder_issue_imm),
        .issue_memsrc   (adder_issue_memsrc),
        .issue_memwrite (),
        .full           (adder_rs_full),
        .alloc1_ok      (adder_rs_alloc1_ok),
        .disp0_alloc_tag(adder_rs_disp0_tag),
        .disp1_alloc_tag(adder_rs_disp1_tag)
    );

    reservation_station #(.TAG_BASE(4), .DEPTH(3)) mul_rs (
        .clk            (clk),
        .reset          (reset),
        .disp0_valid    (disp_valid_0 && (disp_fu_type_0 == FU_MUL_L)),
        .disp0_rd       (disp_rd_0),
        .disp0_alucontrol(3'd0),
        .disp0_memsrc   (1'b0),
        .disp0_imm      (32'd0),
        .disp0_memwrite (1'b0),
        .disp0_val_rs1  (disp_rdata_rs1_0),
        .disp0_val_rs2  (disp_rdata_rs2_0),
        .disp0_tag_rs1  (rat_tag_rs1_0),
        .disp0_tag_rs2  (rat_tag_rs2_0),
        .disp1_valid    (disp_valid_1 && (disp_fu_type_1 == FU_MUL_L)),
        .disp1_rd       (disp_rd_1),
        .disp1_alucontrol(3'd0),
        .disp1_memsrc   (1'b0),
        .disp1_imm      (32'd0),
        .disp1_memwrite (1'b0),
        .disp1_val_rs1  (disp_rdata_rs1_1),
        .disp1_val_rs2  (disp_rdata_rs2_1),
        .disp1_tag_rs1  (rat_tag_rs1_1),
        .disp1_tag_rs2  (rat_tag_rs2_1),
        .cdb_valid      (cdb_valid),
        .cdb_tag        (cdb_tag),
        .cdb_value      (cdb_value),
        .issue_valid    (mul_issue_valid),
        .issue_Vj       (mul_issue_Vj),
        .issue_Vk       (mul_issue_Vk),
        .issue_ctrl     (),
        .issue_tag      (mul_issue_tag),
        .issue_rd       (mul_issue_rd),
        .issue_imm      (),
        .issue_memsrc   (),
        .issue_memwrite (),
        .full           (mul_rs_full),
        .alloc1_ok      (mul_rs_alloc1_ok),
        .disp0_alloc_tag(mul_rs_disp0_tag),
        .disp1_alloc_tag(mul_rs_disp1_tag)
    );

    reservation_station #(.TAG_BASE(7), .DEPTH(3)) mem_rs (
        .clk            (clk),
        .reset          (reset),
        .disp0_valid    (disp_valid_0 && (disp_fu_type_0 == FU_MEM_L)),
        .disp0_rd       (disp_rd_0),
        .disp0_alucontrol(3'd0),
        .disp0_memsrc   (1'b1),
        .disp0_imm      (disp_imm_0),
        .disp0_memwrite (disp_memwrite_0),
        .disp0_val_rs1  (disp_rdata_rs1_0),
        .disp0_val_rs2  (disp_rdata_rs2_0),
        .disp0_tag_rs1  (rat_tag_rs1_0),
        .disp0_tag_rs2  (rat_tag_rs2_0),
        .disp1_valid    (disp_valid_1 && (disp_fu_type_1 == FU_MEM_L)),
        .disp1_rd       (disp_rd_1),
        .disp1_alucontrol(3'd0),
        .disp1_memsrc   (1'b1),
        .disp1_imm      (disp_imm_1),
        .disp1_memwrite (disp_memwrite_1),
        .disp1_val_rs1  (disp_rdata_rs1_1),
        .disp1_val_rs2  (disp_rdata_rs2_1),
        .disp1_tag_rs1  (rat_tag_rs1_1),
        .disp1_tag_rs2  (rat_tag_rs2_1),
        .cdb_valid      (cdb_valid),
        .cdb_tag        (cdb_tag),
        .cdb_value      (cdb_value),
        .issue_valid    (mem_issue_valid),
        .issue_Vj       (mem_issue_Vj),
        .issue_Vk       (mem_issue_Vk),
        .issue_ctrl     (),
        .issue_tag      (mem_issue_tag),
        .issue_rd       (mem_issue_rd),
        .issue_imm      (mem_issue_imm),
        .issue_memsrc   (),
        .issue_memwrite (mem_issue_memwrite),
        .full           (mem_rs_full),
        .alloc1_ok      (mem_rs_alloc1_ok),
        .disp0_alloc_tag(mem_rs_disp0_tag),
        .disp1_alloc_tag(mem_rs_disp1_tag)
    );

    // ================================================================
    // CDB Arbiter
    // ================================================================
    cdb_arbiter u_cdb (
        .clk             (clk),
        .reset           (reset),
        .raw_adder_done  (raw_adder_done),
        .raw_adder_tag   (raw_adder_tag),
        .raw_adder_rd    (raw_adder_rd),
        .raw_adder_result(ooo_adder_result),
        .raw_mul_done    (raw_mul_done),
        .raw_mul_tag     (raw_mul_tag),
        .raw_mul_rd      (raw_mul_rd),
        .raw_mul_result  (ooo_mul_result[31:0]),
        .raw_mem_done    (raw_mem_done),
        .raw_mem_tag     (raw_mem_tag),
        .raw_mem_rd      (raw_mem_rd),
        .raw_mem_result  (ReadDataM),
        .cdb_valid       (cdb_valid),
        .cdb_tag         (cdb_tag),
        .cdb_value       (cdb_value),
        .cdb_rd          (cdb_rd),
        .reg_we          (ooo_reg_we),
        .reg_wa          (ooo_reg_a3),
        .reg_wd          (ooo_reg_wd3)
    );

    // ================================================================
    // FU Tag Pipelines
    //
    // Entry format: {valid[9], rs_tag[8:5], rd[4:0]}  (10 bits)
    // Loaded when the RS issues an instruction to the FU (not at
    // dispatch time).  The valid bit propagates to the output to
    // generate raw_*_done; rs_tag and rd are forwarded to the CDB
    // arbiter which broadcasts the result.
    //
    //   ADDER : alu4_pipeline       = 4-cycle latency
    //   MUL   : mul6_pipeline       = 6-cycle latency
    //   MEM   : agu2_pipeline (2) + SRAM addr reg (1) + SRAM read (1) = 4-cycle latency
    // ================================================================

    // Tag entry: {valid[9], rs_tag[8:5], rd[4:0]}
    logic [9:0] adder_tag [ADDER_LAT-1:0];
    logic [9:0] mul_tag   [MUL_LAT-1:0];
    logic [9:0] mem_tag   [MEM_LAT-1:0];

    // ---- ADDER tag pipeline (4 stages, loaded at RS issue) ----
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < ADDER_LAT; i++) adder_tag[i] <= 10'd0;
        end else begin
            adder_tag[0] <= adder_issue_valid
                            ? {1'b1, adder_issue_tag, adder_issue_rd}
                            : 10'd0;
            for (int i = 1; i < ADDER_LAT; i++)
                adder_tag[i] <= adder_tag[i-1];
        end
    end
    assign raw_adder_done = adder_tag[ADDER_LAT-1][9];
    assign raw_adder_tag  = adder_tag[ADDER_LAT-1][8:5];
    assign raw_adder_rd   = adder_tag[ADDER_LAT-1][4:0];

    // ---- MUL tag pipeline (6 stages, loaded at RS issue) ----
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < MUL_LAT; i++) mul_tag[i] <= 10'd0;
        end else begin
            mul_tag[0] <= mul_issue_valid
                          ? {1'b1, mul_issue_tag, mul_issue_rd}
                          : 10'd0;
            for (int i = 1; i < MUL_LAT; i++)
                mul_tag[i] <= mul_tag[i-1];
        end
    end
    assign raw_mul_done = mul_tag[MUL_LAT-1][9];
    assign raw_mul_tag  = mul_tag[MUL_LAT-1][8:5];
    assign raw_mul_rd   = mul_tag[MUL_LAT-1][4:0];

    // ---- MEM tag pipeline (4 stages, loaded at RS issue) ----
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < MEM_LAT; i++) mem_tag[i] <= 10'd0;
        end else begin
            mem_tag[0] <= mem_issue_valid
                          ? {1'b1, mem_issue_tag, mem_issue_rd}
                          : 10'd0;
            for (int i = 1; i < MEM_LAT; i++)
                mem_tag[i] <= mem_tag[i-1];
        end
    end
    assign raw_mem_done = mem_tag[MEM_LAT-1][9];
    assign raw_mem_tag  = mem_tag[MEM_LAT-1][8:5];
    assign raw_mem_rd   = mem_tag[MEM_LAT-1][4:0];

    // (Writeback arbitration is handled by u_cdb — see cdb_arbiter.sv)

    // ================================================================
    // FU Execution Units
    //
    // Operands come from each RS at issue time, after source operands
    // have been resolved (captured at dispatch or forwarded via CDB).
    // ================================================================

    // ---- ADDER FU (alu4_pipeline, 4-cycle latency) ----
    // SrcB: immediate (memsrc=1, e.g. addi) or Vk (memsrc=0, e.g. add)
    logic [31:0] ooo_adder_a, ooo_adder_b;
    logic [2:0]  ooo_adder_ctrl;
    always_comb begin
        ooo_adder_a    = adder_issue_Vj;
        ooo_adder_b    = adder_issue_memsrc ? adder_issue_imm : adder_issue_Vk;
        ooo_adder_ctrl = adder_issue_ctrl;
    end

    alu4_pipeline u_ooo_adder (
        .clk        (clk),
        .rst        (reset),
        .a          (ooo_adder_a),
        .b          (ooo_adder_b),
        .alucontrol (ooo_adder_ctrl),
        .result     (ooo_adder_result),
        .zero       (),
        .negative   (),
        .v          ()
    );

    // ---- MUL FU (mul6_pipeline, 6-cycle latency) ----
    logic [31:0] ooo_mul_a, ooo_mul_b;
    always_comb begin
        ooo_mul_a = mul_issue_Vj;
        ooo_mul_b = mul_issue_Vk;
    end

    mul6_pipeline u_ooo_mul (
        .clk (clk),
        .rst (reset),
        .a   (ooo_mul_a),
        .b   (ooo_mul_b),
        .c   (ooo_mul_result)
    );

    // ---- MEM FU (agu2_pipeline + synchronous SRAM, 4-cycle latency) ----
    // Cycles 1-2: agu2_pipeline computes rs1+imm (split at 16-bit carry boundary)
    // Cycle 3:    AGU result registered into SRAM address reg (sram_addr_r → DataAdrM)
    //             Side-channel wdata/memwrite also aligned here (mem_sc[2])
    // Cycle 4:    SRAM responds; ReadDataM valid; raw_mem_done fires; result latched

    // ---- MEM FU: operands from MEM RS at issue time ----
    // Vj = rs1 (base addr), imm = offset, Vk = rs2 (write data for stores)
    logic        mem_disp_valid;
    logic [31:0] mem_disp_a, mem_disp_b, mem_disp_wdata;
    logic        mem_disp_memwrite;

    always_comb begin
        mem_disp_valid    = mem_issue_valid;
        mem_disp_a        = mem_issue_Vj;
        mem_disp_b        = mem_issue_imm;
        mem_disp_wdata    = mem_issue_Vk;
        mem_disp_memwrite = mem_issue_memwrite;
    end

    // 2-stage pipelined AGU
    logic [31:0] agu_addr;

    agu2_pipeline u_agu (
        .clk   (clk),
        .rst   (reset),
        .a     (mem_disp_a),
        .b     (mem_disp_b),
        .result(agu_addr)
    );

    // Side-channel pipeline (3 stages, aligned so cycle 3 = SRAM address presented)
    logic        mem_sc_valid    [2:0];
    logic [31:0] mem_sc_wdata    [2:0];
    logic        mem_sc_memwrite [2:0];

    // SRAM address register (cycle 3: register the AGU output into the SRAM)
    logic [31:0] sram_addr_r;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            sram_addr_r        <= 32'd0;
            mem_sc_valid[0]    <= 1'b0; mem_sc_wdata[0] <= 32'd0; mem_sc_memwrite[0] <= 1'b0;
            mem_sc_valid[1]    <= 1'b0; mem_sc_wdata[1] <= 32'd0; mem_sc_memwrite[1] <= 1'b0;
            mem_sc_valid[2]    <= 1'b0; mem_sc_wdata[2] <= 32'd0; mem_sc_memwrite[2] <= 1'b0;
        end else begin
            sram_addr_r        <= agu_addr;
            mem_sc_valid[0]    <= mem_disp_valid;
            mem_sc_wdata[0]    <= mem_disp_wdata;
            mem_sc_memwrite[0] <= mem_disp_memwrite;
            mem_sc_valid[1]    <= mem_sc_valid[0];
            mem_sc_wdata[1]    <= mem_sc_wdata[0];
            mem_sc_memwrite[1] <= mem_sc_memwrite[0];
            mem_sc_valid[2]    <= mem_sc_valid[1];
            mem_sc_wdata[2]    <= mem_sc_wdata[1];
            mem_sc_memwrite[2] <= mem_sc_memwrite[1];
        end
    end

    // SRAM address + write data presented at cycle 3; SRAM responds at cycle 4
    assign DataAdrM   = sram_addr_r;
    assign WriteDataM = mem_sc_wdata[2];
    assign MemWriteM  = mem_sc_valid[2] && mem_sc_memwrite[2];
    assign dispatch_shift_amt = disp_cnt;
    // Note: op, funct3, funct7b5, InstrD are driven by the datapath's
    // legacy pipeline.  Do NOT assign them here — that would create
    // multi-driver conflicts on logic nets.

    datapath dp(
			 .clk(clk),
			 .reset(reset),
			 .RegWrite(RegWriteW),
			 .MemwriteM(dp_MemWriteM),
			 .ImmSrcD(ImmSrcD),
			 .alusrcE(ALUSrcE),
			 .StallD(StallD),
			 .fetch_amt(fetch_amt),
			 .FlushE(FlushE),
			 .FlushD(FlushD),
			 .AluControlE(ALUControlE),
			 .ResultSrcW(ResultSrcW),
			 .ForwardAE(ForwardAE),
			 .ForwardBE(ForwardBE),
			 .InstrF(InstrF),
			 .ReadDataM(ReadDataM),
			 .ZeroE(ZeroE),
			 .op(op),
			 .funct3(funct3),
			 .funct7b5(funct7b5),
			 .InstrD(InstrD),
			 .WriteDataM(dp_WriteDataM),
			 .ResultW(ResultW),
			 .Rs1D(Rs1D),
			 .Rs2D(Rs2D),
			 .Rs1E(Rs1E),
			 .Rs2E(Rs2E),
			 .RdE(RdE),
			 .RdM(RdM),
			 .RdW(RdW),
			 .PCF(PCF),
			 .PCF2(PCF2),
			 .PCSrcE(PCSrcE),
			 .ALUResultM(ALUResultM),
			 // Dispatch-time register reads (OoO path)
			 .disp_rs1_0_dp(disp_rs1_0),
			 .disp_rs2_0_dp(disp_rs2_0),
			 .disp_rs1_1_dp(disp_rs1_1),
			 .disp_rs2_1_dp(disp_rs2_1),
			 .disp_rdata_rs1_0(disp_rdata_rs1_0),
			 .disp_rdata_rs2_0(disp_rdata_rs2_0),
			 .disp_rdata_rs1_1(disp_rdata_rs1_1),
			 .disp_rdata_rs2_1(disp_rdata_rs2_1),
			 // OoO writeback write port (from arbitrator)
			 .ooo_reg_we(ooo_reg_we),
			 .ooo_reg_a3(ooo_reg_a3),
			 .ooo_reg_wd3(ooo_reg_wd3)
		);

    controller ctrl(
        .op(op),
        .funct3(funct3),
        .funct7b5(funct7b5),
        .ZeroE(ZeroE),
        .Negative(Negative),
        .Overflow(Overflow),
        .clk(clk),
		.Reset(reset),
        .FlushE(FlushE),
        .MemWriteE(MemWriteE),
        .MemWriteM(dp_MemWriteM),
        .PCSrcE(PCSrcE),
        .ALUSrcE(ALUSrcE),
        .RegWriteM(RegWriteM),
        .ImmSrcD(ImmSrcD),
        .ALUControlE(ALUControlE),
        .ResultSrcE(ResultSrcE),
        .ResultSrcW(ResultSrcW),
        .RegWriteW(RegWriteW)
    );

    hazardunit h(
        .Rs1D(Rs1D),
        .Rs2D(Rs2D),
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .RdE(RdE),
        .RdM(RdM),
        .RdW(RdW),
        .PCSrcE(PCSrcE),
        .ResultSrcE(ResultSrcE),
        .RegWriteM(RegWriteM),
        .RegWriteW(RegWriteW),
        .StallF(StallF),
        .StallD(StallD),
        .FlushD(FlushD),
        .FlushE(FlushE),
        .ForwardAE(ForwardAE),
        .ForwardBE(ForwardBE),
		.clk(clk),
		.reset(reset)
    );

endmodule