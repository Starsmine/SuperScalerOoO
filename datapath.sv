module datapath(
		input logic clk,
		input logic reset,
		input logic RegWrite, MemwriteM, PCSrcE,
		input logic [2:0] ImmSrcD,
		input logic alusrcE, StallD, FlushE, FlushD,
		input logic [1:0] fetch_amt,             // 0=stall fetch, 1=advance PC by 4, 2=advance by 8
		input logic [2:0] AluControlE,
		input logic [1:0] ResultSrcW, ForwardAE, ForwardBE,
		input logic [31:0] InstrF, ReadDataM,
		// Dispatch-time register read addresses (OoO path)
		input  logic [4:0] disp_rs1_0_dp, disp_rs2_0_dp,  // slot 0
		input  logic [4:0] disp_rs1_1_dp, disp_rs2_1_dp,  // slot 1
		output logic ZeroE,
		output logic [6:0] op,
		output logic [2:0] funct3,
		output logic funct7b5,
		output logic [31:0] InstrD,
		output logic [31:0] WriteDataM, ALUResultM,
	  output logic [31:0] ResultW, PCF, PCF2,  // PCF2 = PCF+4 (second fetch address)
		output logic [4:0] Rs1D, Rs2D, Rs1E,Rs2E, RdE, RdM, RdW,
		// Dispatch-time register read data (OoO path)
		output logic [31:0] disp_rdata_rs1_0, disp_rdata_rs2_0,  // slot 0 operands
		output logic [31:0] disp_rdata_rs1_1, disp_rdata_rs2_1, // slot 1 operands
		// OoO writeback write port (from writeback arbitrator in riscv.sv)
		input  logic        ooo_reg_we,
		input  logic [4:0]  ooo_reg_a3,
		input  logic [31:0] ooo_reg_wd3);
		logic [31:0] PCFN, PCPlus4F, PCTargetE;
		logic [31:0] PCD, PCPlus4D, ImmExtD, ImmExtM, ImmExtW;
		logic [31:0] RD1D, RD2D, RD1E, RD2E; 
		logic [4:0]RdD;
		logic [31:0] ImmExtE, PCPlus4E, WriteDataE, ALUResultE;
		logic [31:0] PCPlus4M, ALUResultW, PCPlus4W;
		logic [31:0] PCE;
		logic [31:0] SrcAE, SrcBE;
		logic [31:0] ReadDataW;
		logic FlushDR;
		


		assign op        = InstrD[6:0];
		assign funct3    = InstrD[14:12];
		assign funct7b5  = InstrD[30];
		assign Rs1D = InstrD[19:15];
		assign Rs2D = InstrD[24:20];
		assign RdD  = InstrD[11:7];
		assign FlushDR = reset || FlushD;


	//next PC logic — advances by 0, 4, or 8 based on fetch_amt
	logic [31:0] PCIncrement, PCSeqNext;
	assign PCIncrement = {28'b0, fetch_amt, 2'b00};  // fetch_amt * 4 (0, 4, or 8)
	assign PCSeqNext   = PCF + PCIncrement;
	assign PCPlus4F    = PCF + 32'd4;  // next sequential word (for return-address / pipeline use)
	assign PCF2        = PCPlus4F;     // exported as second fetch address
	mux2 #(32)  PCADRmux(PCSeqNext, PCTargetE, PCSrcE, PCFN);
	flopre #(32) PcNextFlop(clk, reset, (fetch_amt != 2'b00) | PCSrcE, PCFN, PCF);
	
	
	//idmem block Not in data path

	// FE/ID
	flopre #(32) InstrFeId (clk, FlushDR, !StallD, InstrF, InstrD);
	flopre #(32) PCFeId (clk, FlushDR, !StallD, PCF, PCD);
	flopre #(32) PCPlus4FeId (clk, FlushDR, !StallD, PCPlus4F, PCPlus4D);
	
	//RegBlock
	extend extendblock(InstrD [31:7], ImmSrcD, ImmExtD);
	regfile u_regfile (
		.clk  (clk),
		.reset (reset),
		.we3  (ooo_reg_we),    // OoO arbitrator drives write port
		.a1   (InstrD[19:15]),
		.a2   (InstrD[24:20]),
		.a3   (ooo_reg_a3),
		.wd3  (ooo_reg_wd3),
		.rd1  (RD1D),
		.rd2  (RD2D),
		// Dispatch-time operand reads (OoO path)
		.a4   (disp_rs1_0_dp),
		.a5   (disp_rs2_0_dp),
		.a6   (disp_rs1_1_dp),
		.a7   (disp_rs2_1_dp),
		.rd3  (disp_rdata_rs1_0),
		.rd4  (disp_rdata_rs2_0),
		.rd5  (disp_rdata_rs1_1),
		.rd6  (disp_rdata_rs2_1)
	 );
	 

	// ID/EX 
	flopr #(32) Rd1IdEx (clk, FlushE, RD1D, RD1E);
	flopr #(32) Rd2IdEx (clk, FlushE, RD2D, RD2E);
	flopr #(32) PCIdEx (clk, reset, PCD, PCE);
	flopr #(5) Rs1IdEx (clk, FlushE, Rs1D, Rs1E);
	flopr #(5) Rs2IdEx (clk, FlushE, Rs2D, Rs2E);
	flopr #(5) RddIdEx (clk, reset, RdD, RdE);
	flopr #(32) ImmextIdEx (clk, reset, ImmExtD, ImmExtE);
	flopr #(32) PCPlus4IdEx (clk, reset, PCPlus4D, PCPlus4E);
	
	//aluBlock
	mux3 #(32)  ALUmuxa(RD1E, ResultW, ALUResultM, ForwardAE, SrcAE);
	mux3 #(32)  ALUmuxb(RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE);
	mux2 #(32)  ALUmuxb2(WriteDataE, ImmExtE, alusrcE, SrcBE);
	

  alu4_pipeline u_alu (
    .clk       (clk),
    .rst       (reset),
    .a         (SrcAE),
    .b         (SrcBE),
    .alucontrol(AluControlE),
    .result    (ALUResultE),
    .zero      (ZeroE),
    .negative  (),        
    .v         ()
   );

		
	alu_add_only PCJadder(PCE, ImmExtE, PCTargetE);
	
	// EX/ME 
	flopr #(32) aluResultem(clk, reset, ALUResultE, ALUResultM);
	flopr #(32) writeDataem(clk, reset, WriteDataE, WriteDataM);
	flopr #(5)  rdem(clk, reset, RdE, RdM);
	flopr #(32) pcplus4em(clk, reset, PCPlus4E, PCPlus4M);
	flopr #(32) Immextem (clk, reset, ImmExtE, ImmExtM);

	//dmem
	
	// ME/WB
	
	flopr #(32) readDataMW(clk, reset, ReadDataM, ReadDataW);
	flopr #(32) aluResultMW(clk, reset, ALUResultM, ALUResultW);
	flopr #(5)  rdMW(clk, reset, RdM, RdW);
	flopr #(32) pcPlus4MW(clk, reset, PCPlus4M, PCPlus4W);
	flopr #(32) ImmextMw (clk, reset, ImmExtM, ImmExtW);

	
	
	//wb
	mux4 #(32)  wbmux(ALUResultW, ReadDataW, PCPlus4W, ImmExtW, ResultSrcW, ResultW);
	
	

endmodule



module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module flopre #(parameter WIDTH = 8)
              (input  logic             clk,
               input  logic             reset,
               input  logic             enable,
               input  logic [WIDTH-1:0] d,
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk or posedge reset) begin
    if (reset)        q <= '0;        
    else if (enable)  q <= d;         // capture only when enable is high
  end
endmodule

module flop #(parameter WIDTH = 8)
             (input  logic             clk,
              input  logic [WIDTH-1:0] d,
              output logic [WIDTH-1:0] q);

  always_ff @(posedge clk)
    q <= d;

endmodule



module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2, 
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module mux4 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2, d3,
              input  logic [1:0]       s,
              output logic [WIDTH-1:0] y);

  always_comb begin
    case (s)
      2'b00: y = d0;
      2'b01: y = d1;
      2'b10: y = d2;
      2'b11: y = d3;
      default: y = '0; // safe default
    endcase
  end
endmodule


module extend(input  logic [31:7] instr,
              input  logic [2:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
  case (immsrc)
    // I-type
		3'b000: immext = {{20{instr[31]}}, instr[31:20]};

    // S-type (stores)
		3'b001: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};

    // B-type (branches)
		3'b010: immext = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};

    // J-type (jal)
		3'b011: immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
		
	// U-type (lui)
		3'b100: immext = {instr[31:12], 12'b0};

    default: immext = 32'b0; // undefined
  endcase            
endmodule

module regfile(input  logic        clk, reset,
               input  logic        we3,
               input  logic [ 4:0] a1, a2, a3,    // in-order pipeline ports
               input  logic [31:0] wd3,
               output logic [31:0] rd1, rd2,
               // Dispatch-time read ports (OoO path)
               // slot 0: a4/a5 → rd3/rd4 ; slot 1: a6/a7 → rd5/rd6
               input  logic [ 4:0] a4, a5, a6, a7,
               output logic [31:0] rd3, rd4, rd5, rd6);

  logic [31:0] rf[31:0];

  // Write on rising edge; register 0 hardwired to 0
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      for (int i = 0; i < 32; i++) rf[i] <= 32'b0;
    end else if (we3) begin
      rf[a3] <= wd3;
    end
  end

  // Combinational read helper — x0 always 0, write-bypass for all ports
  function automatic logic [31:0] rd_with_bypass(input logic [4:0] addr);
    if (addr == 5'd0)              return 32'b0;
    if ((addr == a3) && we3)       return wd3;   // write-bypass
    return rf[addr];
  endfunction

  always_comb begin
    rd1 = rd_with_bypass(a1);
    rd2 = rd_with_bypass(a2);
    rd3 = rd_with_bypass(a4);
    rd4 = rd_with_bypass(a5);
    rd5 = rd_with_bypass(a6);
    rd6 = rd_with_bypass(a7);
  end

endmodule

module alu_add_only (
    input  logic [31:0] a,      // first operand
    input  logic [31:0] b,      // second operand
    output logic [31:0] result // sum output
);

    // Perform addition
    assign result = a + b;

endmodule


module alu4_pipeline
        (
        input  logic        clk,
        input  logic        rst,  
        input  logic [31:0] a, b,
        input  logic [2:0]  alucontrol,
        output logic [31:0] result,
        output logic        zero,
			  output logic 		 negative,
			  output logic  		 v);              // overflow);

  logic [31:0] a_s0, a_s1;
  logic [31:0] b_s0, b_s1;
  logic [2:0]  alucontrol_s0, alucontrol_s1;
  logic [31:0] result_s2, result_s3;
  logic        zero_s2, zero_s3;
  logic        negative_s2, negative_s3;
  logic        overflow_s2, overflow_s3;
  logic [31:0] result_calc_s1;
  logic [31:0] condinvb_s1, sum_s1;
  logic        isAddSub_s1;

  // Stage 0-1: pipeline inputs so the ALU latency matches the deeper pipeline.
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      a_s0          <= '0;
      a_s1          <= '0;
      b_s0          <= '0;
      b_s1          <= '0;
      alucontrol_s0 <= '0;
      alucontrol_s1 <= '0;
    end else begin
      a_s0          <= a;
      a_s1          <= a_s0;
      b_s0          <= b;
      b_s1          <= b_s0;
      alucontrol_s0 <= alucontrol;
      alucontrol_s1 <= alucontrol_s0;
    end
  end

  assign condinvb_s1 = alucontrol_s1[0] ? ~b_s1 : b_s1;
  assign sum_s1 = a_s1 + condinvb_s1 + alucontrol_s1[0];
  assign isAddSub_s1 = ((~alucontrol_s1[2]) & (~alucontrol_s1[1])) |
                       ((~alucontrol_s1[1]) & alucontrol_s1[0]);

  always_comb begin
    case (alucontrol_s1)
      3'b000:  result_calc_s1 = sum_s1;                                        // add
      3'b001:  result_calc_s1 = sum_s1;                                        // sub
      3'b010:  result_calc_s1 = a_s1 & b_s1;                                   // and
      3'b011:  result_calc_s1 = a_s1 | b_s1;                                   // or
      3'b100:  result_calc_s1 = a_s1 ^ b_s1;                                   // xor
      3'b101:  result_calc_s1 = ($signed(a_s1) < $signed(b_s1)) ? 32'b1 : 32'b0; // slt
      3'b110:  result_calc_s1 = b_s1 << a_s1[4:0];                             // sll
      3'b111:  result_calc_s1 = b_s1 >> a_s1[4:0];                             // srl
      default: result_calc_s1 = 32'b0;
    endcase
  end

  // Stage 2: evaluate the selected ALU operation and capture aligned flags.
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      result_s2   <= '0;
      zero_s2     <= 1'b0;
      negative_s2 <= 1'b0;
      overflow_s2 <= 1'b0;
    end else begin
      result_s2   <= result_calc_s1;
      zero_s2     <= (result_calc_s1 == 32'b0);
      negative_s2 <= result_calc_s1[31];
      overflow_s2 <= ~(alucontrol_s1[0] ^ a_s1[31] ^ b_s1[31]) &
                     (a_s1[31] ^ sum_s1[31]) &
                     isAddSub_s1;
    end
  end

  // Stage 3: register final outputs.
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      result_s3   <= '0;
      zero_s3     <= 1'b0;
      negative_s3 <= 1'b0;
      overflow_s3 <= 1'b0;
    end else begin
      result_s3   <= result_s2;
      zero_s3     <= zero_s2;
      negative_s3 <= negative_s2;
      overflow_s3 <= overflow_s2;
    end
  end

  assign result   = result_s3;
  assign zero     = zero_s3;
  assign negative = negative_s3;
  assign v        = overflow_s3;
  
endmodule

module mul6_pipeline #(
    parameter W = 32)
    (
    input  logic              clk,
    input  logic              rst,
  input  logic [W-1:0]      a,
  input  logic [W-1:0]      b,
  output logic [2*W-1:0]    c);

    // Stage 0–2: pipeline inputs
  logic [W-1:0] a_s0, a_s1, a_s2;
  logic [W-1:0] b_s0, b_s1, b_s2;

    always_ff @(posedge clk) begin
        if (rst) begin
      a_s0 <= '0; a_s1 <= '0; a_s2 <= '0;
      b_s0 <= '0; b_s1 <= '0; b_s2 <= '0;
        end else begin
      a_s0 <= a;
      a_s1 <= a_s0;
      a_s2 <= a_s1;

            b_s0 <= b;
            b_s1 <= b_s0;
            b_s2 <= b_s1;
        end
    end

    // Stage 3: multiply
    logic [2*W-1:0] prod_s3;
  assign prod_s3 = a_s2 * b_s2;

    // Stages 4–6: pipeline product (3 regs = 6 cycles total: 3 input + comb + 3 output)
    logic [2*W-1:0] prod_s4, prod_s5, prod_s6;

    always_ff @(posedge clk) begin
        if (rst) begin
            prod_s4 <= '0;
            prod_s5 <= '0;
            prod_s6 <= '0;
        end else begin
            prod_s4 <= prod_s3;  // stage 4
            prod_s5 <= prod_s4;  // stage 5
            prod_s6 <= prod_s5;  // stage 6
        end
    end

    // Output available after stage 6 — no extra register, so latency = 6 cycles
    assign c = prod_s6;

endmodule


