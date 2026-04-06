// ============================================================
// cdb_arbiter.sv — Common Data Bus Arbiter
//
// Selects one completing FU per cycle and broadcasts its result
// on the CDB.  The CDB feeds: all reservation stations (snoop),
// the RAT (tag clear), and the regfile (writeback).
//
// Each FU has a pending register.  When the FU's tag pipeline
// indicates completion (raw_*_done), the result + tag + rd are
// latched into the pending register.  Every cycle, the arbiter
// picks the highest-priority pending entry and drives the CDB.
//
// Priority: ADDER > MUL > MEM
//
// Tag encoding (4-bit):
//   0     = no tag / ready
//   1-3   = ADDER RS entries
//   4-6   = MUL   RS entries
//   7-9   = MEM   RS entries
// ============================================================
module cdb_arbiter (
    input  logic        clk,
    input  logic        reset,

    // ---- FU completion inputs (from tag pipelines) ----
    input  logic        raw_adder_done,
    input  logic [3:0]  raw_adder_tag,       // RS tag of completing instr
    input  logic [4:0]  raw_adder_rd,        // destination register
    input  logic [31:0] raw_adder_result,    // computed value

    input  logic        raw_mul_done,
    input  logic [3:0]  raw_mul_tag,
    input  logic [4:0]  raw_mul_rd,
    input  logic [31:0] raw_mul_result,

    input  logic        raw_mem_done,
    input  logic [3:0]  raw_mem_tag,
    input  logic [4:0]  raw_mem_rd,
    input  logic [31:0] raw_mem_result,

    // ---- CDB broadcast output ----
    output logic        cdb_valid,
    output logic [3:0]  cdb_tag,
    output logic [31:0] cdb_value,
    output logic [4:0]  cdb_rd,              // for regfile writeback

    // ---- Regfile write port (directly driven from CDB) ----
    output logic        reg_we,
    output logic [4:0]  reg_wa,
    output logic [31:0] reg_wd
);

    // ------------------------------------------------------------------
    // Pending registers (one per FU)
    // ------------------------------------------------------------------
    logic        adder_pend_v;
    logic [3:0]  adder_pend_tag;
    logic [4:0]  adder_pend_rd;
    logic [31:0] adder_pend_data;

    logic        mul_pend_v;
    logic [3:0]  mul_pend_tag;
    logic [4:0]  mul_pend_rd;
    logic [31:0] mul_pend_data;

    logic        mem_pend_v;
    logic [3:0]  mem_pend_tag;
    logic [4:0]  mem_pend_rd;
    logic [31:0] mem_pend_data;

    // ------------------------------------------------------------------
    // Arbitration: ADDER > MUL > MEM
    // ------------------------------------------------------------------
    wire adder_wins = adder_pend_v;
    wire mul_wins   = !adder_pend_v &&  mul_pend_v;
    wire mem_wins   = !adder_pend_v && !mul_pend_v && mem_pend_v;

    // ------------------------------------------------------------------
    // CDB outputs
    // ------------------------------------------------------------------
    assign cdb_valid = adder_wins | mul_wins | mem_wins;

    assign cdb_tag   = adder_wins ? adder_pend_tag  :
                       mul_wins   ? mul_pend_tag    : mem_pend_tag;

    assign cdb_value = adder_wins ? adder_pend_data :
                       mul_wins   ? mul_pend_data   : mem_pend_data;

    assign cdb_rd    = adder_wins ? adder_pend_rd   :
                       mul_wins   ? mul_pend_rd     : mem_pend_rd;

    // ------------------------------------------------------------------
    // Regfile write port (CDB drives it directly)
    // ------------------------------------------------------------------
    assign reg_we = cdb_valid && (cdb_rd != 5'd0);
    assign reg_wa = cdb_rd;
    assign reg_wd = cdb_value;

    // ------------------------------------------------------------------
    // Pending register update
    //
    // Set when raw_*_done fires (FU tag pipeline completes).
    // Clear when the arbiter selects this FU (*_wins).
    //
    // If raw_*_done and *_wins fire in the same cycle, the new
    // result takes priority (set overwrites clear).
    // ------------------------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            adder_pend_v    <= 1'b0;
            adder_pend_tag  <= 4'd0;
            adder_pend_rd   <= 5'd0;
            adder_pend_data <= 32'd0;

            mul_pend_v      <= 1'b0;
            mul_pend_tag    <= 4'd0;
            mul_pend_rd     <= 5'd0;
            mul_pend_data   <= 32'd0;

            mem_pend_v      <= 1'b0;
            mem_pend_tag    <= 4'd0;
            mem_pend_rd     <= 5'd0;
            mem_pend_data   <= 32'd0;
        end else begin

            // ADDER pending register
            if (raw_adder_done) begin
                adder_pend_v    <= 1'b1;
                adder_pend_tag  <= raw_adder_tag;
                adder_pend_rd   <= raw_adder_rd;
                adder_pend_data <= raw_adder_result;
            end else if (adder_wins) begin
                adder_pend_v <= 1'b0;
            end

            // MUL pending register
            if (raw_mul_done) begin
                mul_pend_v    <= 1'b1;
                mul_pend_tag  <= raw_mul_tag;
                mul_pend_rd   <= raw_mul_rd;
                mul_pend_data <= raw_mul_result;
            end else if (mul_wins) begin
                mul_pend_v <= 1'b0;
            end

            // MEM pending register
            if (raw_mem_done) begin
                mem_pend_v    <= 1'b1;
                mem_pend_tag  <= raw_mem_tag;
                mem_pend_rd   <= raw_mem_rd;
                mem_pend_data <= raw_mem_result;
            end else if (mem_wins) begin
                mem_pend_v <= 1'b0;
            end

        end
    end

endmodule
