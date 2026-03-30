// ============================================================
// scoreboard.sv  —  CDC 6600-style in-order issue scoreboard
//
// Supports two dispatch slots (in-order) and three functional
// unit classes:
//   FU_ADDER (2'b01) — 4-cycle 
//   FU_MUL   (2'b10) — 6-cycle 
//   FU_MEM   (2'b11) — 1-cycle (combinational dmem read/write)
//   FU_NONE  (2'b00) — NOP, no FU needed
//
// Detects and stalls on:
//   Structural  : target FU is occupied by a previous instruction
//   RAW         : source register is produced by an in-flight FU
//   WAW         : destination register already claimed by in-flight FU
//   Intra-pair  : hazards between slot-0 and slot-1 in same cycle
//
// WAR is not a hazard: operands are read at issue time, before
// any future instruction can overwrite them.
// ============================================================
module scoreboard (
    input  logic clk,
    input  logic reset,

    // ---- Dispatch slot 0 (combinational query from dispatch_unit) ----
    input  logic        q0_valid,        // instruction present in slot 0
    input  logic [4:0]  q0_rd,           // destination register
    input  logic [4:0]  q0_rs1, q0_rs2, // source registers
    input  logic [1:0]  q0_fu_type,      // which FU this instruction needs

    // ---- Dispatch slot 1 (combinational query from dispatch_unit) ----
    input  logic        q1_valid,
    input  logic [4:0]  q1_rd,
    input  logic [4:0]  q1_rs1, q1_rs2,
    input  logic [1:0]  q1_fu_type,

    // ---- Writeback completions (one-cycle pulse when FU finishes) ----
    input  logic        adder_done,      // adder result written to regfile this cycle
    input  logic [4:0]  adder_wb_rd,
    input  logic        mul_done,
    input  logic [4:0]  mul_wb_rd,
    input  logic        mem_done,
    input  logic [4:0]  mem_wb_rd,

    // ---- Hazard / stall outputs (combinational) ----
    output logic        stall0,          // slot 0 must NOT issue this cycle
    output logic        stall1,          // slot 1 must NOT issue this cycle

    // ---- FU busy status (for external observation / debug) ----
    output logic        adder_busy,
    output logic        mul_busy,
    output logic        mem_busy
);

    // ------------------------------------------------------------------
    // FU type encoding — must match dispatch_unit fu_type outputs
    // ------------------------------------------------------------------
    localparam logic [1:0] FU_NONE  = 2'b00;
    localparam logic [1:0] FU_ADDER = 2'b01;
    localparam logic [1:0] FU_MUL   = 2'b10;
    localparam logic [1:0] FU_MEM   = 2'b11;

    // ------------------------------------------------------------------
    // Registered state
    // ------------------------------------------------------------------

    // FU busy flags — set at issue, cleared at writeback
    logic adder_busy_r, mul_busy_r, mem_busy_r;
    assign adder_busy = adder_busy_r;
    assign mul_busy   = mul_busy_r;
    assign mem_busy   = mem_busy_r;

    // Register result status table:
    //   reg_fu[i] = FU_NONE  → register i is ready (no one will write it)
    //   reg_fu[i] = FU_ADDER → adder has this register in-flight
    //   reg_fu[i] = FU_MUL   → multiplier has this register in-flight
    //   reg_fu[i] = FU_MEM   → memory FU has this register in-flight
    logic [1:0] reg_fu [31:0];

    // ------------------------------------------------------------------
    // Combinational hazard detection (inlined, no function calls)
    //
    // Using always_comb instead of assign+function to ensure the
    // simulator properly tracks sensitivity on reg_fu[] array
    // elements accessed via variable indices.
    // ------------------------------------------------------------------
    logic s0_struct, s0_raw, s0_waw;
    logic s1_struct, s1_raw, s1_waw;
    logic issue0_commits;
    logic intra_struct, intra_raw, intra_waw;

    always_comb begin
        // --- Slot 0 hazards ---

        // Structural: FU busy
        case (q0_fu_type)
            FU_ADDER: s0_struct = q0_valid && adder_busy_r;
            FU_MUL:   s0_struct = q0_valid && mul_busy_r;
            FU_MEM:   s0_struct = q0_valid && mem_busy_r;
            default:  s0_struct = 1'b0;
        endcase

        // RAW: source register produced by in-flight FU
        s0_raw = q0_valid && (
            ((q0_rs1 != 5'd0) && (reg_fu[q0_rs1] != FU_NONE)) ||
            ((q0_rs2 != 5'd0) && (reg_fu[q0_rs2] != FU_NONE))
        );

        // WAW: destination register claimed by in-flight FU
        s0_waw = q0_valid && (q0_rd != 5'd0) && (reg_fu[q0_rd] != FU_NONE);

        stall0 = s0_struct | s0_raw | s0_waw;

        // --- Intra-pair hazards (slot 0 → slot 1) ---
        issue0_commits = q0_valid && !stall0 && (q0_fu_type != FU_NONE);

        intra_struct = issue0_commits &&
                       (q1_fu_type != FU_NONE) &&
                       (q0_fu_type == q1_fu_type);

        intra_raw = issue0_commits &&
                    (q0_rd != 5'd0) &&
                    ((q0_rd == q1_rs1) || (q0_rd == q1_rs2));

        intra_waw = issue0_commits &&
                    (q0_rd != 5'd0) &&
                    (q0_rd == q1_rd);

        // --- Slot 1 hazards ---
        case (q1_fu_type)
            FU_ADDER: s1_struct = q1_valid && (adder_busy_r || intra_struct);
            FU_MUL:   s1_struct = q1_valid && (mul_busy_r   || intra_struct);
            FU_MEM:   s1_struct = q1_valid && (mem_busy_r   || intra_struct);
            default:  s1_struct = q1_valid && intra_struct;
        endcase

        s1_raw = q1_valid && (
            ((q1_rs1 != 5'd0) && (reg_fu[q1_rs1] != FU_NONE)) ||
            ((q1_rs2 != 5'd0) && (reg_fu[q1_rs2] != FU_NONE)) ||
            intra_raw
        );

        s1_waw = q1_valid && (q1_rd != 5'd0) &&
                 ((reg_fu[q1_rd] != FU_NONE) || intra_waw);

        // In-order: if slot 0 stalls, slot 1 cannot issue either
        stall1 = stall0 | s1_struct | s1_raw | s1_waw;
    end

    // ------------------------------------------------------------------
    // Issue commit signals (used locally to update state)
    // ------------------------------------------------------------------
    logic do_issue0, do_issue1;
    assign do_issue0 = q0_valid && !stall0;
    assign do_issue1 = q1_valid && !stall1;

    // ------------------------------------------------------------------
    // Registered state update
    // ------------------------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            adder_busy_r <= 1'b0;
            mul_busy_r   <= 1'b0;
            mem_busy_r   <= 1'b0;
            for (int i = 0; i < 32; i++) reg_fu[i] <= FU_NONE;
        end else begin

            // ----------------------------------------------------------
            // Writeback: release FU and register ownership
            // (Processed before issue so a completing FU becomes free
            //  for potential next-cycle re-issue without an extra bubble)
            // ----------------------------------------------------------
            if (adder_done) begin
                adder_busy_r <= 1'b0;
                if (adder_wb_rd != 5'd0 && reg_fu[adder_wb_rd] == FU_ADDER)
                    reg_fu[adder_wb_rd] <= FU_NONE;
            end
            if (mul_done) begin
                mul_busy_r <= 1'b0;
                if (mul_wb_rd != 5'd0 && reg_fu[mul_wb_rd] == FU_MUL)
                    reg_fu[mul_wb_rd] <= FU_NONE;
            end
            if (mem_done) begin
                mem_busy_r <= 1'b0;
                if (mem_wb_rd != 5'd0 && reg_fu[mem_wb_rd] == FU_MEM)
                    reg_fu[mem_wb_rd] <= FU_NONE;
            end

            // ----------------------------------------------------------
            // Issue: mark FU busy and claim destination register
            // ----------------------------------------------------------
            if (do_issue0 && q0_fu_type != FU_NONE) begin
                case (q0_fu_type)
                    FU_ADDER: adder_busy_r <= 1'b1;
                    FU_MUL:   mul_busy_r   <= 1'b1;
                    FU_MEM:   mem_busy_r   <= 1'b1;
                    default:  ;
                endcase
                if (q0_rd != 5'd0)
                    reg_fu[q0_rd] <= q0_fu_type;
            end

            if (do_issue1 && q1_fu_type != FU_NONE) begin
                case (q1_fu_type)
                    FU_ADDER: adder_busy_r <= 1'b1;
                    FU_MUL:   mul_busy_r   <= 1'b1;
                    FU_MEM:   mem_busy_r   <= 1'b1;
                    default:  ;
                endcase
                if (q1_rd != 5'd0)
                    reg_fu[q1_rd] <= q1_fu_type;
            end

        end
    end

endmodule
