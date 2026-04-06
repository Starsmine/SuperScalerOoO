module dispatch_unit (
    input  logic [31:0]      instr0, instr1,
    input  logic [1:0]       num_valid,
    // Stall inputs from scoreboard (hazard detection)
    input  logic             stall0_in,   // scoreboard says slot 0 cannot issue
    input  logic             stall1_in,   // scoreboard says slot 1 cannot issue
    
    // Decoded outputs for instruction 0
    output logic [6:0]       op0,
    output logic [2:0]       funct3_0,
    output logic             funct7b5_0,
    output logic [4:0]       rs1_0, rs2_0, rd_0,
    output logic [31:0]      imm_0,
    output logic [2:0]       alucontrol_0,
    output logic [1:0]       aluop_0,
    output logic             regwrite_0, memwrite_0, memsrc_0,
    output logic [1:0]       fu_type_0,   // FU_NONE/ADDER/MUL/MEM for scoreboard
    output logic             valid_0,
    
    // Decoded outputs for instruction 1
    output logic [6:0]       op1,
    output logic [2:0]       funct3_1,
    output logic             funct7b5_1,
    output logic [4:0]       rs1_1, rs2_1, rd_1,
    output logic [31:0]      imm_1,
    output logic [2:0]       alucontrol_1,
    output logic [1:0]       aluop_1,
    output logic             regwrite_1, memwrite_1, memsrc_1,
    output logic [1:0]       fu_type_1,
    output logic             valid_1,
    
    // Dispatch signals
    output logic [1:0]       dispatch_cnt    // number of instructions dispatched
);

    // Extract fields from instruction 0
    assign op0       = instr0[6:0];
    assign funct3_0  = instr0[14:12];
    assign funct7b5_0 = instr0[30];
    assign rs1_0     = instr0[19:15];
    assign rs2_0     = instr0[24:20];
    // For stores, instr[11:7] encodes imm[4:0], not a destination register.
    // Zero rd when the instruction does not write back to the register file.
    assign rd_0      = regwrite_0 ? instr0[11:7] : 5'd0;
    
    // Extract fields from instruction 1
    assign op1       = instr1[6:0];
    assign funct3_1  = instr1[14:12];
    assign funct7b5_1 = instr1[30];
    assign rs1_1     = instr1[19:15];
    assign rs2_1     = instr1[24:20];
    assign rd_1      = regwrite_1 ? instr1[11:7] : 5'd0;
    
    // Internal wires for ImmSrc and ALUSrc from maindec
    logic [2:0] immsrc_0, immsrc_1;
    logic       alusrc_0, alusrc_1;

    // Decode control signals for instruction 0
    maindec md0(
        .op(op0),
        .ResultSrc(),
        .MemWrite(memwrite_0),
        .Branch(),
        .ALUSrc(alusrc_0),
        .RegWrite(regwrite_0),
        .Jump(),
        .ImmSrc(immsrc_0),
        .ALUOp(aluop_0)
    );
    
    aludec ad0(
        .opb5(op0[5]),
        .funct3(funct3_0),
        .funct7b5(funct7b5_0),
        .ALUOp(aluop_0),
        .ALUControl(alucontrol_0)
    );
    
    // Decode control signals for instruction 1
    maindec md1(
        .op(op1),
        .ResultSrc(),
        .MemWrite(memwrite_1),
        .Branch(),
        .ALUSrc(alusrc_1),
        .RegWrite(regwrite_1),
        .Jump(),
        .ImmSrc(immsrc_1),
        .ALUOp(aluop_1)
    );
    
    aludec ad1(
        .opb5(op1[5]),
        .funct3(funct3_1),
        .funct7b5(funct7b5_1),
        .ALUOp(aluop_1),
        .ALUControl(alucontrol_1)
    );
    
    // memsrc = ALUSrc: indicates the instruction uses an immediate (load/store/I-type)
    assign memsrc_0 = alusrc_0;
    assign memsrc_1 = alusrc_1;
    
    // Immediate extension using the proper ImmSrc from maindec
    // Scope: I-type (lw, addi, etc.) -> 3'b000, S-type (sw) -> 3'b001, NOP -> default 0
    extend ext0(instr0[31:7], immsrc_0, imm_0);
    extend ext1(instr1[31:7], immsrc_1, imm_1);

    // ------------------------------------------------------------------
    // FU type classification
    // Must match scoreboard.sv localparam encoding:
    //   FU_NONE=2'b00, FU_ADDER=2'b01, FU_MUL=2'b10, FU_MEM=2'b11
    // MUL is R-type (op=0110011) with funct7[0]=instr[25]=1 (RISC-V M ext)
    // ------------------------------------------------------------------
    always_comb begin
        // Instruction 0
        // rd==x0 means no visible side-effect for ALU/MUL → treat as NOP
        if      (op0 == 7'b0110011 && instr0[25] && instr0[11:7] != 5'd0)
                                                      fu_type_0 = 2'b10; // MUL
        else if (op0 == 7'b0000011 || op0 == 7'b0100011)
                                                      fu_type_0 = 2'b11; // load/store -> MEM
        else if ((op0 == 7'b0110011 || op0 == 7'b0010011) && instr0[11:7] != 5'd0)
                                                      fu_type_0 = 2'b01; // R-type/I-type -> ADDER
        else                                          fu_type_0 = 2'b00; // NOP / other

        // Instruction 1
        if      (op1 == 7'b0110011 && instr1[25] && instr1[11:7] != 5'd0)
                                                      fu_type_1 = 2'b10;
        else if (op1 == 7'b0000011 || op1 == 7'b0100011)
                                                      fu_type_1 = 2'b11;
        else if ((op1 == 7'b0110011 || op1 == 7'b0010011) && instr1[11:7] != 5'd0)
                                                      fu_type_1 = 2'b01;
        else                                          fu_type_1 = 2'b00;
    end

    // ------------------------------------------------------------------
    // Dispatch logic
    // num_valid is a count (0, 1, 2) — not a bitmask.
    // Stalls come entirely from the scoreboard (structural/RAW/WAW).
    // In-order rule: if slot 0 stalls, slot 1 cannot issue (stall1_in
    // already incorporates stall0_in from the scoreboard).
    // ------------------------------------------------------------------
    always_comb begin
        valid_0 = (num_valid >= 2'd1) && !stall0_in;
        valid_1 = (num_valid >= 2'd2) && !stall1_in;
        dispatch_cnt = {1'b0, valid_0} + {1'b0, valid_1};
    end

endmodule
