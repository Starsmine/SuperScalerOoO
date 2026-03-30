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

    // Scoreboard signals
    logic        sb_stall0, sb_stall1;
    logic        sb_adder_busy, sb_mul_busy, sb_mem_busy;
    // Writeback done pulses (driven from tag pipelines)
    logic        adder_done, mul_done, mem_done;
    logic [4:0]  adder_wb_rd, mul_wb_rd, mem_wb_rd;

    // Decouple in-order controller/datapath signals from OoO output ports
    logic        dp_MemWriteM;      // controller → datapath (in-order path)
    logic [31:0] dp_WriteDataM;     // datapath store data (in-order path, unused by OoO)

    // OoO FU result signals
    logic [31:0] ooo_adder_result;
    logic [63:0] ooo_mul_result;

    // Raw FU completions (tag pipeline output, before arbitration delay)
    logic        raw_adder_done, raw_mul_done, raw_mem_done;

    // OoO regfile write port (driven by writeback arbitrator)
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
        .stall0_in(sb_stall0),
        .stall1_in(sb_stall1),
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

    // Scoreboard — CDC 6600-style hazard detection
    scoreboard sb(
        .clk(clk),
        .reset(reset),
        // Slot 0 query
        .q0_valid(queue_valid_cnt >= 2'd1),
        .q0_rd(disp_rd_0),
        .q0_rs1(disp_rs1_0),
        .q0_rs2(disp_rs2_0),
        .q0_fu_type(disp_fu_type_0),
        // Slot 1 query
        .q1_valid(queue_valid_cnt >= 2'd2),
        .q1_rd(disp_rd_1),
        .q1_rs1(disp_rs1_1),
        .q1_rs2(disp_rs2_1),
        .q1_fu_type(disp_fu_type_1),
        // Writeback completions (from FU tag pipelines)
        .adder_done(adder_done),
        .adder_wb_rd(adder_wb_rd),
        .mul_done(mul_done),
        .mul_wb_rd(mul_wb_rd),
        .mem_done(mem_done),
        .mem_wb_rd(mem_wb_rd),
        // Hazard outputs
        .stall0(sb_stall0),
        .stall1(sb_stall1),
        // Status
        .adder_busy(sb_adder_busy),
        .mul_busy(sb_mul_busy),
        .mem_busy(sb_mem_busy)
    );

    // ================================================================
    // FU Tag Pipelines
    //
    // Each shift register tracks in-flight instructions through a FU
    // and generates the scoreboard's done pulse + writeback rd address.
    //
    //   ADDER : alu4_pipeline       = 4-cycle latency
    //   MUL   : mul6_pipeline       = 6-cycle latency
    //   MEM   : addr (4) + 2 flops  = 6-cycle latency (lw)
    //           Stores: rd = x0, so mem_done fires but clears no register.
    //
    // The scoreboard guarantees at most one instruction per FU per cycle
    // (structural hazard detection), so no arbitration is needed here.
    // ================================================================
    localparam logic [1:0] FU_NONE_L  = 2'b00;
    localparam logic [1:0] FU_ADDER_L = 2'b01;
    localparam logic [1:0] FU_MUL_L   = 2'b10;
    localparam logic [1:0] FU_MEM_L   = 2'b11;

    localparam int ADDER_LAT = 4;
    localparam int MUL_LAT   = 6;
    localparam int MEM_LAT   = 2;  // 1 capture reg + 1 result latch

    // Tag entry format: {valid[5], rd_addr[4:0]}
    logic [5:0] adder_tag [ADDER_LAT-1:0];
    logic [5:0] mul_tag   [MUL_LAT-1:0];
    logic [5:0] mem_tag   [MEM_LAT-1:0];

    // ---- Adder tag pipeline (4 stages) ----
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < ADDER_LAT; i++) adder_tag[i] <= 6'd0;
        end else begin
            adder_tag[0] <= (disp_valid_0 && (disp_fu_type_0 == FU_ADDER_L)) ? {1'b1, disp_rd_0} :
                            (disp_valid_1 && (disp_fu_type_1 == FU_ADDER_L)) ? {1'b1, disp_rd_1} :
                            6'd0;
            for (int i = 1; i < ADDER_LAT; i++)
                adder_tag[i] <= adder_tag[i-1];
        end
    end
    assign raw_adder_done = adder_tag[ADDER_LAT-1][5];

    // ---- Mul tag pipeline (6 stages) ----
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < MUL_LAT; i++) mul_tag[i] <= 6'd0;
        end else begin
            mul_tag[0] <= (disp_valid_0 && (disp_fu_type_0 == FU_MUL_L)) ? {1'b1, disp_rd_0} :
                          (disp_valid_1 && (disp_fu_type_1 == FU_MUL_L)) ? {1'b1, disp_rd_1} :
                          6'd0;
            for (int i = 1; i < MUL_LAT; i++)
                mul_tag[i] <= mul_tag[i-1];
        end
    end
    assign raw_mul_done = mul_tag[MUL_LAT-1][5];

    // ---- Mem tag pipeline (2 stages) ----
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < MEM_LAT; i++) mem_tag[i] <= 6'd0;
        end else begin
            mem_tag[0] <= (disp_valid_0 && (disp_fu_type_0 == FU_MEM_L)) ? {1'b1, disp_rd_0} :
                          (disp_valid_1 && (disp_fu_type_1 == FU_MEM_L)) ? {1'b1, disp_rd_1} :
                          6'd0;
            for (int i = 1; i < MEM_LAT; i++)
                mem_tag[i] <= mem_tag[i-1];
        end
    end
    assign raw_mem_done = mem_tag[MEM_LAT-1][5];

    // ================================================================
    // Writeback Arbitrator
    //
    // Each FU has one pending slot.  When the tag pipeline fires
    // (raw_*_done) the result is latched into a pending register.
    // Every cycle the arbitrator selects the highest-priority pending
    // result, writes it to the register file, and THEN pulses the
    // scoreboard done signal.  The scoreboard therefore does NOT
    // release reg_fu[rd] until the data is actually written, keeping
    // RAW hazard detection correct even when arbitration stalls a
    // result by one cycle.
    //
    // Priority:  ADDER (2) > MUL (3) > MEM (6)  [cycles, in-flight]
    // ================================================================

    // --- Pending registers (one per FU) ---
    logic        adder_pend_v, mul_pend_v, mem_pend_v;
    logic [4:0]  adder_pend_rd, mul_pend_rd, mem_pend_rd;
    logic [31:0] adder_pend_data, mul_pend_data, mem_pend_data;

    // --- Arbitration: ADDER beats MUL beats MEM ---
    logic adder_wins, mul_wins, mem_wins;
    assign adder_wins = adder_pend_v;
    assign mul_wins   = !adder_pend_v &&  mul_pend_v;
    assign mem_wins   = !adder_pend_v && !mul_pend_v && mem_pend_v;

    // --- OoO regfile write port: winner drives it ---
    assign ooo_reg_we  = adder_wins | mul_wins | mem_wins;
    assign ooo_reg_a3  = adder_wins ? adder_pend_rd   :
                         mul_wins   ? mul_pend_rd     : mem_pend_rd;
    assign ooo_reg_wd3 = adder_wins ? adder_pend_data :
                         mul_wins   ? mul_pend_data   : mem_pend_data;

    // --- Scoreboard done = cycle result is actually written ---
    assign adder_done  = adder_wins;  assign adder_wb_rd = adder_pend_rd;
    assign mul_done    = mul_wins;    assign mul_wb_rd   = mul_pend_rd;
    assign mem_done    = mem_wins;    assign mem_wb_rd   = mem_pend_rd;

    // --- Pending register update ---
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            adder_pend_v <= 1'b0; adder_pend_rd <= '0; adder_pend_data <= '0;
            mul_pend_v   <= 1'b0; mul_pend_rd   <= '0; mul_pend_data   <= '0;
            mem_pend_v   <= 1'b0; mem_pend_rd   <= '0; mem_pend_data   <= '0;
        end else begin
            // ADDER: set on raw done, clear when arbitrator writes it.
            // raw_adder_done && adder_wins cannot coincide because the
            // scoreboard keeps adder_busy until adder_wins fires.
            if (raw_adder_done) begin
                adder_pend_v    <= 1'b1;
                adder_pend_rd   <= adder_tag[ADDER_LAT-1][4:0];
                adder_pend_data <= ooo_adder_result;  // registered output valid this cycle
            end else if (adder_wins) begin
                adder_pend_v <= 1'b0;
            end

            // MUL
            if (raw_mul_done) begin
                mul_pend_v    <= 1'b1;
                mul_pend_rd   <= mul_tag[MUL_LAT-1][4:0];
                mul_pend_data <= ooo_mul_result[31:0]; // lower 32 bits (RV32 mul)
            end else if (mul_wins) begin
                mul_pend_v <= 1'b0;
            end

            // MEM: DataAdrM = mem_fu_addr_r is already driving the memory
            // combinationally, so ReadDataM is valid this same cycle.
            if (raw_mem_done) begin
                mem_pend_v    <= 1'b1;
                mem_pend_rd   <= mem_tag[MEM_LAT-1][4:0];
                mem_pend_data <= ReadDataM; // combinational, correct this cycle
            end else if (mem_wins) begin
                mem_pend_v <= 1'b0;
            end
        end
    end

    // ================================================================
    // FU Execution Units
    //
    // Operands are read from the regfile at dispatch time (disp_rdata_*).
    // They are muxed combinationally into each FU's input; the FU's own
    // first pipeline register serves as the capture register.
    //
    // MEM FU uses an explicit capture register because it is not a
    // standard pipeline module — it drives the external memory interface.
    // ================================================================

    // ---- ADDER FU (alu4_pipeline, 4-cycle latency) ----
    logic [31:0] ooo_adder_a, ooo_adder_b;
    logic [2:0]  ooo_adder_ctrl;
    always_comb begin
        if (disp_valid_0 && (disp_fu_type_0 == FU_ADDER_L)) begin
            ooo_adder_a    = disp_rdata_rs1_0;
            ooo_adder_b    = disp_memsrc_0 ? disp_imm_0 : disp_rdata_rs2_0;
            ooo_adder_ctrl = disp_alucontrol_0;
        end else if (disp_valid_1 && (disp_fu_type_1 == FU_ADDER_L)) begin
            ooo_adder_a    = disp_rdata_rs1_1;
            ooo_adder_b    = disp_memsrc_1 ? disp_imm_1 : disp_rdata_rs2_1;
            ooo_adder_ctrl = disp_alucontrol_1;
        end else begin
            ooo_adder_a    = '0;
            ooo_adder_b    = '0;
            ooo_adder_ctrl = '0;
        end
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
        if (disp_valid_0 && (disp_fu_type_0 == FU_MUL_L)) begin
            ooo_mul_a = disp_rdata_rs1_0;
            ooo_mul_b = disp_rdata_rs2_0;
        end else if (disp_valid_1 && (disp_fu_type_1 == FU_MUL_L)) begin
            ooo_mul_a = disp_rdata_rs1_1;
            ooo_mul_b = disp_rdata_rs2_1;
        end else begin
            ooo_mul_a = '0;
            ooo_mul_b = '0;
        end
    end

    mul6_pipeline u_ooo_mul (
        .clk (clk),
        .rst (reset),
        .a   (ooo_mul_a),
        .b   (ooo_mul_b),
        .c   (ooo_mul_result)
    );

    // ---- MEM FU (capture register + external memory, 2-cycle latency) ----
    // Cycle 1: capture addr (rs1+imm), wdata (rs2), memwrite; drive DataAdrM.
    // Cycle 2: latch ReadDataM (combinational from tb) into ooo_mem_result.
    logic        mem_fu_valid_r;
    logic [31:0] mem_fu_addr_r;
    logic [31:0] mem_fu_wdata_r;
    logic        mem_fu_memwrite_r;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_fu_valid_r    <= 1'b0;
            mem_fu_addr_r     <= '0;
            mem_fu_wdata_r    <= '0;
            mem_fu_memwrite_r <= 1'b0;
        end else begin
            if (disp_valid_0 && (disp_fu_type_0 == FU_MEM_L)) begin
                mem_fu_valid_r    <= 1'b1;
                mem_fu_addr_r     <= disp_rdata_rs1_0 + disp_imm_0;
                mem_fu_wdata_r    <= disp_rdata_rs2_0;
                mem_fu_memwrite_r <= disp_memwrite_0;
            end else if (disp_valid_1 && (disp_fu_type_1 == FU_MEM_L)) begin
                mem_fu_valid_r    <= 1'b1;
                mem_fu_addr_r     <= disp_rdata_rs1_1 + disp_imm_1;
                mem_fu_wdata_r    <= disp_rdata_rs2_1;
                mem_fu_memwrite_r <= disp_memwrite_1;
            end else begin
                mem_fu_valid_r    <= 1'b0;
                mem_fu_memwrite_r <= 1'b0;
            end
        end
    end

    // Override legacy placeholder and drive external memory ports from OoO MEM FU
    // (The 'assign DataAdrM = ALUResultM' above is superseded by these)
    assign DataAdrM  = mem_fu_addr_r;
    assign WriteDataM = mem_fu_wdata_r;
    assign MemWriteM  = mem_fu_valid_r && mem_fu_memwrite_r;
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