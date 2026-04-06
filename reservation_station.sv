// ============================================================
// reservation_station.sv — Single Tomasulo Reservation Station
//
// Parameterized module: one instance per FU.  Instantiate three
// times in the top level with different TAG_BASE values:
//   ADDER RS: TAG_BASE = 1  -> entry tags 1, 2, 3
//   MUL   RS: TAG_BASE = 4  -> entry tags 4, 5, 6
//   MEM   RS: TAG_BASE = 7  -> entry tags 7, 8, 9
//
// Tag 0 = "ready" -- the operand value is already known.
// Tags are 4 bits wide (range 0-9).
//
// Each RS is DEPTH entries deep (default 3).
//
// Dual-dispatch: up to 2 instructions per cycle can be inserted
// if both dispatch slots target this RS's FU type and there are
// enough free entries.  The top-level gates disp0/1_valid so
// that only instructions matching this FU arrive here.
//
// CDB snoop: every cycle, if cdb_valid, entries whose Qj or Qk
// matches cdb_tag fill the corresponding V field and clear Q to 0.
//
// Issue: the lowest-index ready entry (Qj==0 && Qk==0) fires to
// the FU each cycle.  After issue, the entry is freed.
// ============================================================
module reservation_station #(
    parameter int TAG_BASE = 1,
    parameter int DEPTH    = 3
) (
    input  logic        clk,
    input  logic        reset,

    // ---- Dispatch slot 0 (top-level gates valid by FU type) ----
    input  logic        disp0_valid,
    input  logic [4:0]  disp0_rd,
    input  logic [2:0]  disp0_alucontrol,
    input  logic        disp0_memsrc,
    input  logic [31:0] disp0_imm,
    input  logic        disp0_memwrite,
    input  logic [31:0] disp0_val_rs1,
    input  logic [31:0] disp0_val_rs2,
    input  logic [3:0]  disp0_tag_rs1,
    input  logic [3:0]  disp0_tag_rs2,

    // ---- Dispatch slot 1 ----
    input  logic        disp1_valid,
    input  logic [4:0]  disp1_rd,
    input  logic [2:0]  disp1_alucontrol,
    input  logic        disp1_memsrc,
    input  logic [31:0] disp1_imm,
    input  logic        disp1_memwrite,
    input  logic [31:0] disp1_val_rs1,
    input  logic [31:0] disp1_val_rs2,
    input  logic [3:0]  disp1_tag_rs1,
    input  logic [3:0]  disp1_tag_rs2,

    // ---- CDB snoop ----
    input  logic        cdb_valid,
    input  logic [3:0]  cdb_tag,
    input  logic [31:0] cdb_value,

    // ---- Issue output (active one cycle when an entry fires) ----
    output logic        issue_valid,
    output logic [31:0] issue_Vj,
    output logic [31:0] issue_Vk,
    output logic [2:0]  issue_ctrl,
    output logic [3:0]  issue_tag,
    output logic [4:0]  issue_rd,
    output logic [31:0] issue_imm,
    output logic        issue_memsrc,
    output logic        issue_memwrite,

    // ---- Status / allocation ----
    output logic        full,             // no free entries at all
    output logic        alloc1_ok,        // slot 1 can allocate (considering slot 0 claim)
    output logic [3:0]  disp0_alloc_tag,  // 0 = not allocated
    output logic [3:0]  disp1_alloc_tag   // 0 = not allocated
);

    // ------------------------------------------------------------------
    // Entry fields  (3 entries, indices 0-2)
    // ------------------------------------------------------------------
    logic        rs_busy     [2:0];
    logic        rs_issued   [2:0];   // in-flight: issued but not yet CDB'd
    logic [2:0]  rs_op       [2:0];
    logic [31:0] rs_Vj       [2:0];
    logic [31:0] rs_Vk       [2:0];
    logic [3:0]  rs_Qj       [2:0];
    logic [3:0]  rs_Qk       [2:0];
    logic [4:0]  rs_rd       [2:0];
    logic [31:0] rs_imm      [2:0];
    logic        rs_memsrc   [2:0];
    logic        rs_memwrite [2:0];

    // ------------------------------------------------------------------
    // Helper: local index (0-2) -> global tag
    // ------------------------------------------------------------------
    function automatic logic [3:0] idx_to_tag(input int idx);
        return 4'(TAG_BASE + idx);
    endfunction

    // ------------------------------------------------------------------
    // Free-entry detection & full flag
    // ------------------------------------------------------------------
    wire free0 = !rs_busy[0];
    wire free1 = !rs_busy[1];
    wire free2 = !rs_busy[2];

    assign full = !free0 && !free1 && !free2;

    // ------------------------------------------------------------------
    // Allocation -- slot 0 gets first free, slot 1 gets next free
    // ------------------------------------------------------------------
    logic [1:0] alloc0_idx;
    logic       alloc0_ok;
    logic [1:0] alloc1_idx;
    logic s0c0, s0c1, s0c2;  // slot-0 claims each RS entry this cycle

    always_comb begin : alloc_slot0
        alloc0_idx = 2'd0;
        alloc0_ok  = 1'b0;
        if      (free0) begin alloc0_idx = 2'd0; alloc0_ok = 1'b1; end
        else if (free1) begin alloc0_idx = 2'd1; alloc0_ok = 1'b1; end
        else if (free2) begin alloc0_idx = 2'd2; alloc0_ok = 1'b1; end
    end

    always_comb begin : alloc_slot1
        alloc1_idx = 2'd0;
        alloc1_ok  = 1'b0;

        s0c0 = disp0_valid && alloc0_ok && (alloc0_idx == 2'd0);
        s0c1 = disp0_valid && alloc0_ok && (alloc0_idx == 2'd1);
        s0c2 = disp0_valid && alloc0_ok && (alloc0_idx == 2'd2);

        if      (free0 && !s0c0) begin alloc1_idx = 2'd0; alloc1_ok = 1'b1; end
        else if (free1 && !s0c1) begin alloc1_idx = 2'd1; alloc1_ok = 1'b1; end
        else if (free2 && !s0c2) begin alloc1_idx = 2'd2; alloc1_ok = 1'b1; end
    end

    assign disp0_alloc_tag = (disp0_valid && alloc0_ok) ? idx_to_tag(int'(alloc0_idx)) : 4'd0;
    assign disp1_alloc_tag = (disp1_valid && alloc1_ok) ? idx_to_tag(int'(alloc1_idx)) : 4'd0;

    // ------------------------------------------------------------------
    // Issue selection -- lowest-index ready entry
    // ------------------------------------------------------------------
    wire ready0 = rs_busy[0] && !rs_issued[0] && (rs_Qj[0] == 4'd0) && (rs_Qk[0] == 4'd0);
    wire ready1 = rs_busy[1] && !rs_issued[1] && (rs_Qj[1] == 4'd0) && (rs_Qk[1] == 4'd0);
    wire ready2 = rs_busy[2] && !rs_issued[2] && (rs_Qj[2] == 4'd0) && (rs_Qk[2] == 4'd0);

    logic [1:0] issue_idx;

    always_comb begin
        issue_valid = 1'b0;
        issue_idx   = 2'd0;
        if      (ready0) begin issue_valid = 1'b1; issue_idx = 2'd0; end
        else if (ready1) begin issue_valid = 1'b1; issue_idx = 2'd1; end
        else if (ready2) begin issue_valid = 1'b1; issue_idx = 2'd2; end
    end

    // ------------------------------------------------------------------
    // Issue outputs -- raw fields; top-level computes FU-specific
    // signals (e.g. ADDER srcB mux, MEM address add).
    // ------------------------------------------------------------------
    always_comb begin
        issue_Vj       = rs_Vj      [issue_idx];
        issue_Vk       = rs_Vk      [issue_idx];
        issue_ctrl     = rs_op       [issue_idx];
        issue_tag      = idx_to_tag(int'(issue_idx));
        issue_rd       = rs_rd       [issue_idx];
        issue_imm      = rs_imm      [issue_idx];
        issue_memsrc   = rs_memsrc   [issue_idx];
        issue_memwrite = rs_memwrite [issue_idx];
    end

    // ------------------------------------------------------------------
    // Sequential update
    //   Priority: 1) Issue clear  2) CDB snoop  3) Dispatch fill
    // ------------------------------------------------------------------
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < DEPTH; i++) begin
                rs_busy[i]     <= 1'b0;
                rs_issued[i]   <= 1'b0;
                rs_op[i]       <= 3'd0;
                rs_Vj[i]       <= 32'd0;
                rs_Vk[i]       <= 32'd0;
                rs_Qj[i]       <= 4'd0;
                rs_Qk[i]       <= 4'd0;
                rs_rd[i]       <= 5'd0;
                rs_imm[i]      <= 32'd0;
                rs_memsrc[i]   <= 1'b0;
                rs_memwrite[i] <= 1'b0;
            end
        end else begin

            // ---- Step 1: Issue — mark in-flight (entry stays busy) ----
            if (issue_valid)
                rs_issued[issue_idx] <= 1'b1;

            // ---- Step 2: CDB snoop + entry free ----
            if (cdb_valid && cdb_tag != 4'd0) begin
                for (int i = 0; i < DEPTH; i++) begin
                    // Free entry whose result has arrived on CDB
                    if (rs_issued[i] && cdb_tag == idx_to_tag(i)) begin
                        rs_busy[i]   <= 1'b0;
                        rs_issued[i] <= 1'b0;
                    end
                    // Snoop: wake up waiting operands
                    if (rs_busy[i] && !rs_issued[i]) begin
                        if (rs_Qj[i] == cdb_tag) begin
                            rs_Qj[i] <= 4'd0;
                            rs_Vj[i] <= cdb_value;
                        end
                        if (rs_Qk[i] == cdb_tag) begin
                            rs_Qk[i] <= 4'd0;
                            rs_Vk[i] <= cdb_value;
                        end
                    end
                end
            end

            // ---- Step 3: Dispatch -- slot 0 ----
            if (disp0_valid && alloc0_ok) begin
                rs_busy    [alloc0_idx] <= 1'b1;
                rs_op      [alloc0_idx] <= disp0_alucontrol;
                rs_rd      [alloc0_idx] <= disp0_rd;
                rs_imm     [alloc0_idx] <= disp0_imm;
                rs_memsrc  [alloc0_idx] <= disp0_memsrc;
                rs_memwrite[alloc0_idx] <= disp0_memwrite;

                if (disp0_tag_rs1 == 4'd0) begin
                    rs_Qj[alloc0_idx] <= 4'd0;
                    rs_Vj[alloc0_idx] <= disp0_val_rs1;
                end else if (cdb_valid && disp0_tag_rs1 == cdb_tag) begin
                    rs_Qj[alloc0_idx] <= 4'd0;
                    rs_Vj[alloc0_idx] <= cdb_value;
                end else begin
                    rs_Qj[alloc0_idx] <= disp0_tag_rs1;
                    rs_Vj[alloc0_idx] <= 32'd0;
                end

                if (disp0_tag_rs2 == 4'd0) begin
                    rs_Qk[alloc0_idx] <= 4'd0;
                    rs_Vk[alloc0_idx] <= disp0_val_rs2;
                end else if (cdb_valid && disp0_tag_rs2 == cdb_tag) begin
                    rs_Qk[alloc0_idx] <= 4'd0;
                    rs_Vk[alloc0_idx] <= cdb_value;
                end else begin
                    rs_Qk[alloc0_idx] <= disp0_tag_rs2;
                    rs_Vk[alloc0_idx] <= 32'd0;
                end
            end

            // ---- Dispatch -- slot 1 ----
            if (disp1_valid && alloc1_ok) begin
                rs_busy    [alloc1_idx] <= 1'b1;
                rs_op      [alloc1_idx] <= disp1_alucontrol;
                rs_rd      [alloc1_idx] <= disp1_rd;
                rs_imm     [alloc1_idx] <= disp1_imm;
                rs_memsrc  [alloc1_idx] <= disp1_memsrc;
                rs_memwrite[alloc1_idx] <= disp1_memwrite;

                if (disp1_tag_rs1 == 4'd0) begin
                    rs_Qj[alloc1_idx] <= 4'd0;
                    rs_Vj[alloc1_idx] <= disp1_val_rs1;
                end else if (cdb_valid && disp1_tag_rs1 == cdb_tag) begin
                    rs_Qj[alloc1_idx] <= 4'd0;
                    rs_Vj[alloc1_idx] <= cdb_value;
                end else begin
                    rs_Qj[alloc1_idx] <= disp1_tag_rs1;
                    rs_Vj[alloc1_idx] <= 32'd0;
                end

                if (disp1_tag_rs2 == 4'd0) begin
                    rs_Qk[alloc1_idx] <= 4'd0;
                    rs_Vk[alloc1_idx] <= disp1_val_rs2;
                end else if (cdb_valid && disp1_tag_rs2 == cdb_tag) begin
                    rs_Qk[alloc1_idx] <= 4'd0;
                    rs_Vk[alloc1_idx] <= cdb_value;
                end else begin
                    rs_Qk[alloc1_idx] <= disp1_tag_rs2;
                    rs_Vk[alloc1_idx] <= 32'd0;
                end
            end

        end
    end

endmodule
