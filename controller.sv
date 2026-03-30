module controller(input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       ZeroE, Negative, Overflow,
						input  logic 		 clk, Reset,
						input  logic		 FlushE,
                  output logic       MemWriteE,
						output logic 		 MemWriteM, 
                  output logic       PCSrcE, ALUSrcE,
                  output logic       RegWriteM,
                  output logic [2:0] ImmSrcD,
                  output logic [2:0] ALUControlE,
						output logic [1:0] ResultSrcE,
						output logic [1:0] ResultSrcW,
						output logic       RegWriteW);

  logic [1:0] ALUOp;
  logic [2:0] ALUControlD;
  logic [1:0] ResultSrcD;
  logic       MemWriteD, BranchD, ALUSrcD, RegWriteD, JumpD;
  logic       RegWriteE, BranchE, JumpE;
  logic [1:0] ResultSrcM;
  logic FlushER;

  // Decode stage control
  maindec md(
    .op(op),
    .ResultSrc(ResultSrcD),
    .MemWrite(MemWriteD),
    .Branch(BranchD),
    .ALUSrc(ALUSrcD),
    .RegWrite(RegWriteD),
    .Jump(JumpD),
    .ImmSrc(ImmSrcD),
    .ALUOp(ALUOp)
  );

  // ALU decode
  aludec ad(
    .opb5(op[5]),
    .funct3(funct3),
    .funct7b5(funct7b5),
    .ALUOp(ALUOp),
    .ALUControl(ALUControlD)
  );


  assign FlushER = FlushE || Reset;
  
  	// ID/EX
	control_d2e ctrlD2E (
    .clk(clk),
	 .reset(FlushER),
    .RegWriteD(RegWriteD),
    .ResultSrcD(ResultSrcD),
    .MemWriteD(MemWriteD),
    .JumpD(JumpD),
    .BranchD(BranchD),
    .ALUControlD(ALUControlD),
    .ALUSrcD(ALUSrcD),
    .RegWriteE(RegWriteE),
    .ResultSrcE(ResultSrcE),
    .MemWriteE(MemWriteE),
    .JumpE(JumpE),
    .BranchE(BranchE),
    .ALUControlE(ALUControlE),
    .ALUSrcE(ALUSrcE)
);
	
	// EX/ME
	control_e2m ctrlE2M (
    .clk(clk),
	 .reset(Reset),
    .RegWriteE(RegWriteE),
    .ResultSrcE(ResultSrcE),
    .MemWriteE(MemWriteE),
    .RegWriteM(RegWriteM),
    .ResultSrcM(ResultSrcM),
    .MemWriteM(MemWriteM)
);

	
	// ME/WB
	control_m2w ctrlM2W (
    .clk(clk),
	 .reset(Reset),
    .RegWriteM(RegWriteM),
    .ResultSrcM(ResultSrcM),
    .RegWriteW(RegWriteW),
    .ResultSrcW(ResultSrcW)
);

  logic PCSrcE_reg;
		//assign PCSrcE = PCSrcE_reg;
		assign PCSrcE = (JumpE === 1'b1) || ((ZeroE === 1'b1) && (BranchE === 1'b1));

//		always_ff @(posedge clk or posedge Reset) begin
//			 if (Reset)
//				  PCSrcE_reg <= 1'b0;
//			 else
			//	PCSrcE_reg <= (JumpE === 1'b1) || ((ZeroE === 1'b1) && (BranchE === 1'b1));
		//end
		

endmodule


module control_d2e (
    input  logic clk,
    input  logic reset,   // use FlushE as reset
    input  logic RegWriteD, MemWriteD, JumpD, BranchD, 
    input  logic [1:0] ResultSrcD,
    input  logic [2:0] ALUControlD,
    input  logic       ALUSrcD,
    output logic RegWriteE, MemWriteE, JumpE, BranchE,
    output logic [1:0] ResultSrcE,
    output logic [2:0] ALUControlE,
    output logic       ALUSrcE
);

  always_ff @(posedge clk) begin
    if (reset) begin
      // squash all E-stage control signals
      RegWriteE   <= 1'b0;
      ResultSrcE  <= 2'b00;
      MemWriteE   <= 1'b0;
      JumpE       <= 1'b0;
      BranchE     <= 1'b0;
      ALUControlE <= 3'b000;
      ALUSrcE     <= 1'b0;
    end else begin
      // normal pipeline advance
      RegWriteE   <= RegWriteD;
      ResultSrcE  <= ResultSrcD;
      MemWriteE   <= MemWriteD;
      JumpE       <= JumpD;
      BranchE     <= BranchD;
      ALUControlE <= ALUControlD;
      ALUSrcE     <= ALUSrcD;
    end
  end

endmodule


module control_e2m (
    input  logic clk,
    input  logic reset,
    input  logic RegWriteE, MemWriteE,
    input  logic [1:0] ResultSrcE,
    output logic RegWriteM, MemWriteM,
    output logic [1:0] ResultSrcM
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteM  <= 1'b0;
      MemWriteM  <= 1'b0;
      ResultSrcM <= 2'b00;
    end else begin
      RegWriteM  <= RegWriteE; 
		MemWriteM  <= MemWriteE;
      ResultSrcM <= ResultSrcE;
    end
  end

endmodule


module control_m2w (
    input  logic clk,
    input  logic reset,
    input  logic RegWriteM,
    input  logic [1:0] ResultSrcM,
    output logic RegWriteW,
    output logic [1:0] ResultSrcW
);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      RegWriteW  <= 1'b0;
      ResultSrcW <= 2'b00;
    end else begin
      RegWriteW <= RegWriteM; 
      ResultSrcW <= ResultSrcM;
    end
  end

endmodule


module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [2:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [11:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
			// RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 12'b1_000_1_0_01_0_00_0; // lw
      /////////////////////////////////////////////////
      // Write more operators here                   //
      7'b0100011: controls = 12'b0_001_1_1_00_0_00_0; // sw
      7'b0110011: controls = 12'b1_000_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 12'b0_010_0_0_00_1_01_0; // beq/bge
      7'b0010011: controls = 12'b1_000_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 12'b1_011_0_0_10_0_00_1; // jal
		7'b0110111: controls = 12'b1_100_1_0_11_0_00_0; // lui 
      /////////////////////////////////////////////////
      default: controls = 12'b0_000_0_0_00_0_00_0; // safe default // non-implemented instruction
    endcase
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
                 3'b010:    ALUControl = 3'b101; // slt, slti
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b111:    ALUControl = 3'b010; // and, andi
                 default:   ALUControl = 3'b000; // ???
               endcase
    endcase
endmodule



