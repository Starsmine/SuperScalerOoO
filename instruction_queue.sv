module instruction_queue #(
    parameter DEPTH = 8,  // number of instruction slots
    parameter WIDTH = 32
) (
    input  logic                   clk, reset,
    // Fetch inputs: up to 2 instructions pushed per cycle
    input  logic [WIDTH-1:0]       instr_in0,      // instruction at PCF
    input  logic [WIDTH-1:0]       instr_in1,      // instruction at PCF+4 (used when fetch_amt==2)
    input  logic [1:0]             fetch_amt,       // how many instructions to push this cycle (0-2)
    // Dispatch drain
    input  logic [1:0]             shift_amt,       // how many instructions consumed by dispatch (0-2)
    // Outputs
    output logic [WIDTH-1:0]       instr_out0,     // oldest instruction
    output logic [WIDTH-1:0]       instr_out1,     // second oldest instruction
    output logic [1:0]             num_valid,      // valid instructions at head (0-2)
    output logic [1:0]             space_avail     // free slots available for fetch this cycle (0-2)
);

    // FIFO-style queue
    logic [WIDTH-1:0] queue [DEPTH-1:0];
    logic [3:0] head, tail;
    logic [3:0] count;  // how many instructions are in queue
    
    wire empty = (count == 0);
    wire full = (count == DEPTH);
    
    // Number of valid outputs (0, 1, or 2 instructions available)
    wire [3:0] avail = count > 2 ? 4'd2 : count;
    assign num_valid = avail[1:0];

    // Free slots for fetch this cycle (accounts for slots dispatch will free, capped at 2)
    wire [4:0] free_slots = (DEPTH - count) + {3'b0, shift_amt};
    assign space_avail = (free_slots >= 5'd2) ? 2'd2 : free_slots[1:0];

    // Output the first two instructions in queue
    assign instr_out0 = queue[head];
    assign instr_out1 = queue[(head + 1) % DEPTH];
    
    // Push fetched instructions and drain dispatched instructions each cycle
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            head  <= 4'b0;
            tail  <= 4'b0;
            count <= 4'b0;
            for (int i = 0; i < DEPTH; i++) queue[i] <= 32'b0;
        end else begin
            // Push up to 2 fetched instructions into tail
            if (fetch_amt >= 1)
                queue[tail] <= instr_in0;
            if (fetch_amt >= 2)
                queue[(tail + 1) % DEPTH] <= instr_in1;

            // Update pointers and count atomically
            head  <= (head + shift_amt) % DEPTH;
            tail  <= (tail  + fetch_amt) % DEPTH;
            count <= count - shift_amt + fetch_amt;
        end
    end

endmodule
