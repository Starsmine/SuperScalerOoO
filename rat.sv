// ============================================================
// rat.sv  --  Register Alias Table (RAT) for Tomasulo's Algorithm
//
// Tracks which in-flight RS entry will produce each architectural
// register's next value.  A tag of 0 means the register is ready
// (its value is already in the architectural register file).
//
// Tag encoding (4-bit, matches reservation_station.sv):
//   4'd0          = ready -- value is in the register file
//   4'd1 - 4'd3   = ADDER RS entries 0-2
//   4'd4 - 4'd6   = MUL   RS entries 0-2
//   4'd7 - 4'd9   = MEM   RS entries 0-2
//
// Two in-order dispatch slots per cycle.  Slot 1's source reads
// include a combinational bypass for the case where slot 0 is
// simultaneously writing the same destination register (intra-pair
// RAW).
//
// CDB broadcast clears any RAT entry whose stored tag equals the
// completing tag (the value has been written to the register file).
// If a dispatch in the same cycle re-writes that same register, the
// dispatch takes priority (last assignment in always_ff wins).
//
// x0 is never given a tag: reads of x0 always return 4'd0, and
// writes to x0 (rd == 5'd0) are silently ignored.
// ============================================================
module rat (
    input  logic        clk,
    input  logic        reset,

    // ---- Dispatch slot 0 ----------------------------------------
    input  logic        disp0_en,
    input  logic [4:0]  disp0_rd,
    input  logic [3:0]  disp0_tag,

    input  logic [4:0]  disp0_rs1,
    input  logic [4:0]  disp0_rs2,
    output logic [3:0]  disp0_tag_rs1,
    output logic [3:0]  disp0_tag_rs2,

    // ---- Dispatch slot 1 ----------------------------------------
    input  logic        disp1_en,
    input  logic [4:0]  disp1_rd,
    input  logic [3:0]  disp1_tag,

    input  logic [4:0]  disp1_rs1,
    input  logic [4:0]  disp1_rs2,
    output logic [3:0]  disp1_tag_rs1,
    output logic [3:0]  disp1_tag_rs2,

    // ---- Common Data Bus (CDB) broadcast ------------------------
    input  logic        cdb_valid,
    input  logic [3:0]  cdb_tag,

    // ---- Full RAT state ----------------------------------------
    output logic [3:0]  rat_out [31:0]
);

    logic [3:0] rat_r [31:0];

    genvar g;
    generate
        for (g = 0; g < 32; g++) begin : rat_export
            assign rat_out[g] = rat_r[g];
        end
    endgenerate

    // Slot 0 -- straight registered lookup (x0 always 0)
    assign disp0_tag_rs1 = (disp0_rs1 == 5'd0) ? 4'd0 : rat_r[disp0_rs1];
    assign disp0_tag_rs2 = (disp0_rs2 == 5'd0) ? 4'd0 : rat_r[disp0_rs2];

    // Slot 1 -- registered lookup + slot-0 bypass
    always_comb begin
        if (disp1_rs1 == 5'd0)
            disp1_tag_rs1 = 4'd0;
        else if (disp0_en && (disp0_rd != 5'd0) && (disp0_rd == disp1_rs1))
            disp1_tag_rs1 = disp0_tag;
        else
            disp1_tag_rs1 = rat_r[disp1_rs1];

        if (disp1_rs2 == 5'd0)
            disp1_tag_rs2 = 4'd0;
        else if (disp0_en && (disp0_rd != 5'd0) && (disp0_rd == disp1_rs2))
            disp1_tag_rs2 = disp0_tag;
        else
            disp1_tag_rs2 = rat_r[disp1_rs2];
    end

    // Sequential update -- priority (last assignment wins):
    //   1. CDB clear    2. Slot 0 alloc    3. Slot 1 alloc
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 32; i++) rat_r[i] <= 4'd0;
        end else begin

            if (cdb_valid && cdb_tag != 4'd0) begin
                for (int i = 1; i < 32; i++) begin
                    if (rat_r[i] == cdb_tag)
                        rat_r[i] <= 4'd0;
                end
            end

            if (disp0_en && disp0_rd != 5'd0)
                rat_r[disp0_rd] <= disp0_tag;

            if (disp1_en && disp1_rd != 5'd0)
                rat_r[disp1_rd] <= disp1_tag;

        end
    end

endmodule
