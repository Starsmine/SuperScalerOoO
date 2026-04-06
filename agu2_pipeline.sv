// ============================================================
// agu2_pipeline.sv  —  2-Stage Pipelined Address Generation Unit
//
// Computes a + b (rs1 + sign-extended immediate) for load/store
// address calculation, pipelined in 2 stages why 
//
// The 32-bit addition is split at the 16-bit carry boundary:
//   Stage 1: lower 16-bit sum + carry-out  (registered)
//            upper halves forwarded         (registered)
//   Stage 2: upper 16-bit sum + carry-in  (registered)
//            concatenated with lower sum → full 32-bit result
//
// Output 'result' is a registered signal, available 2 clock
// cycles after the inputs are presented.
// ============================================================
module agu2_pipeline (
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] a,       // rs1 (base register value)
    input  logic [31:0] b,       // sign-extended immediate offset
    output logic [31:0] result   // a + b, registered, 2 cycles latency
);

    // ------------------------------------------------------------------
    // Stage 1: lower half sum + carry, forward upper halves
    // ------------------------------------------------------------------
    logic [16:0] s1_lo;    // [16] = carry-out from lower 16-bit add
    logic [15:0] s1_a_hi;  // a[31:16] forwarded to stage 2
    logic [15:0] s1_b_hi;  // b[31:16] forwarded to stage 2

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            s1_lo   <= 17'd0;
            s1_a_hi <= 16'd0;
            s1_b_hi <= 16'd0;
        end else begin
            s1_lo   <= {1'b0, a[15:0]} + {1'b0, b[15:0]};
            s1_a_hi <= a[31:16];
            s1_b_hi <= b[31:16];
        end
    end

    // ------------------------------------------------------------------
    // Stage 2: upper half sum using registered carry-in
    // ------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 32'd0;
        end else begin
            result <= {s1_a_hi + s1_b_hi + {15'd0, s1_lo[16]},
                       s1_lo[15:0]};
        end
    end

endmodule
