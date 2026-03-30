module hazardunit (
    input  logic        clk,
    input  logic        reset,
    input  logic [4:0]  Rs1D, Rs2D,       // Source registers in Decode stage
    input  logic [4:0]  Rs1E, Rs2E, RdE,  // Source and destination registers in Execute stage
    input  logic [4:0]  RdM, RdW,         // Destination registers in Memory and Writeback stages
    input  logic        PCSrcE,
    input  logic [1:0]  ResultSrcE,
    input  logic        RegWriteM, RegWriteW, // Write enables for M/W stages
    output logic        StallF, StallD,  // Stall signals for Fetch and Decode
    output logic        FlushD, FlushE,  // Flush signals for Decode and Execute
    output logic [1:0]  ForwardAE, ForwardBE // Forwarding controls for ALU inputs
);

    // Forwarding logic for SrcAE
    always_comb begin
        if ((Rs1E === RdM) && RegWriteM && (Rs1E != 5'd0))
            ForwardAE = 2'b10;
        else if ((Rs1E === RdW) && RegWriteW && (Rs1E != 5'd0))
            ForwardAE = 2'b01;
        else
            ForwardAE = 2'b00;
    end

    // Forwarding logic for SrcBE
    always_comb begin
        if ((Rs2E === RdM) && RegWriteM && (Rs2E != 5'd0))
            ForwardBE = 2'b10;
        else if ((Rs2E === RdW) && RegWriteW && (Rs2E != 5'd0))
            ForwardBE = 2'b01;
        else
            ForwardBE = 2'b00;
    end

    // Registered load-use hazard stall
		 logic lwStall_comb;
		 
	assign lwStall_comb = (ResultSrcE == 2'b01) &&
								 ((Rs1D === RdE) || (Rs2D === RdE));

	// Drive stalls and flush from the combinational detect
	assign StallF = lwStall_comb;
	assign StallD = lwStall_comb;
	

    // Control hazard flush
    always_comb begin
        FlushD = PCSrcE;
        FlushE = lwStall_comb || PCSrcE;
    end

endmodule