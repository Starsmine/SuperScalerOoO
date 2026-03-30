`timescale 1ns/1ps

// =============================================================================
// Testbench: tb_riscv
//
// Self-checking baseline suite for the superscalar OoO core.
// Each scenario rewrites imem, resets the DUT, runs long enough for the
// pipeline/FUs to drain, and then checks architectural side effects.
// =============================================================================
module tb_riscv;

    localparam logic [31:0] NOP = 32'h00000013;
    localparam bit DEBUG_TRACE = 1'b0;

    // -----------------------------------------------------------------------
    // DUT ports
    // -----------------------------------------------------------------------
    logic        clk, reset;
    logic [31:0] ReadDataM, InstrF, InstrF2;
    logic [31:0] PCF, PCF2;
    logic        MemWriteM;
    logic [31:0] DataAdrM, WriteDataM;

    // -----------------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------------
    riscv dut (
        .clk       (clk),
        .reset     (reset),
        .ReadDataM (ReadDataM),
        .InstrF    (InstrF),
        .InstrF2   (InstrF2),
        .PCF       (PCF),
        .PCF2      (PCF2),
        .MemWriteM (MemWriteM),
        .DataAdrM  (DataAdrM),
        .WriteDataM(WriteDataM)
    );

    // -----------------------------------------------------------------------
    // Clock: 10 ns period
    // -----------------------------------------------------------------------
    initial clk = 0;
    always  #5 clk = ~clk;

    // -----------------------------------------------------------------------
    // Instruction memory (ROM) — 16-word word-addressed
    // Rewritten per test case.
    // -----------------------------------------------------------------------
    logic [31:0] imem [0:15];

    // Feed instructions based on PC (word-addressed, ignore lower 2 bits)
    assign InstrF  = imem[PCF[5:2]];
    assign InstrF2 = imem[PCF2[5:2]];  // second fetch: always PCF+4

    // -----------------------------------------------------------------------
    // Data memory (RAM) — 64-word word-addressed
    // -----------------------------------------------------------------------
    logic [31:0] dmem [0:63];

    // Write on clock edge when MemWriteM is asserted; zero on reset.
    // Initialisation lives inside the always_ff so dmem has only one driver.
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 64; i++) dmem[i] <= 32'b0;
        end else if (MemWriteM) begin
            dmem[DataAdrM[7:2]] <= WriteDataM;
        end
    end

    // Combinational read
    assign ReadDataM = dmem[DataAdrM[7:2]];

    // -----------------------------------------------------------------------
    // Monitor — print every cycle for visibility
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!reset && DEBUG_TRACE) begin
            if ($time == 35000)
                $display("Cycle  |  PCF         | MemWriteM | DataAdrM | WriteDataM\n-------+------+-------+-----------+----------+-----------");
            $display("%6t ns | PC=%08h | MWr=%b | Adr=%08h | Wdat=%08h",
                     $time, PCF, MemWriteM, DataAdrM, WriteDataM);
            // OoO dispatch debug
            $display("         Q: valid_cnt=%0d head_instr=%08h,%08h",
                     dut.queue_valid_cnt, dut.instr_queue0, dut.instr_queue1);
            $display("         DISP: v0=%b fu0=%0d rd0=%0d rs1_0=%0d rs2_0=%0d | v1=%b fu1=%0d rd1=%0d | cnt=%0d",
                     dut.disp_valid_0, dut.disp_fu_type_0, dut.disp_rd_0,
                     dut.disp_rs1_0, dut.disp_rs2_0,
                     dut.disp_valid_1, dut.disp_fu_type_1, dut.disp_rd_1,
                     dut.disp_cnt);
            $display("         SB: stall0=%b stall1=%b | busy A=%b M=%b Mem=%b",
                     dut.sb_stall0, dut.sb_stall1,
                     dut.sb_adder_busy, dut.sb_mul_busy, dut.sb_mem_busy);
            $display("         SB_DETAIL: s0_struct=%b s0_raw=%b s0_waw=%b | reg_fu[1]=%0d reg_fu[2]=%0d reg_fu[3]=%0d",
                     dut.sb.s0_struct, dut.sb.s0_raw, dut.sb.s0_waw,
                     dut.sb.reg_fu[1], dut.sb.reg_fu[2], dut.sb.reg_fu[3]);
            $display("         SB_ISSUE: do_issue0=%b do_issue1=%b | adder_busy_r=%b q0_valid=%b q0_fu=%0d q0_rd=%0d",
                     dut.sb.do_issue0, dut.sb.do_issue1,
                     dut.sb.adder_busy_r, dut.sb.q0_valid, dut.sb.q0_fu_type, dut.sb.q0_rd);
            $display("         TAG: raw_a=%b raw_m=%b raw_mem=%b | pend_a=%b pend_m=%b pend_mem=%b",
                     dut.raw_adder_done, dut.raw_mul_done, dut.raw_mem_done,
                     dut.adder_pend_v, dut.mul_pend_v, dut.mem_pend_v);
            $display("         WB: we=%b a3=%0d wd3=%0d | adder_done=%b mul_done=%b mem_done=%b",
                     dut.ooo_reg_we, dut.ooo_reg_a3, dut.ooo_reg_wd3,
                     dut.adder_done, dut.mul_done, dut.mem_done);
            $display("         MEM_FU: valid=%b addr=%08h wdata=%08h mwr=%b",
                     dut.mem_fu_valid_r, dut.mem_fu_addr_r, dut.mem_fu_wdata_r, dut.mem_fu_memwrite_r);
            $display("         RDATA: rs1_0=%0d rs2_0=%0d rs1_1=%0d rs2_1=%0d",
                     dut.disp_rdata_rs1_0, dut.disp_rdata_rs2_0,
                     dut.disp_rdata_rs1_1, dut.disp_rdata_rs2_1);
            $display("---");
        end
    end

    // -----------------------------------------------------------------------
    // Stimulus and self-check
    // -----------------------------------------------------------------------
    int errors = 0;
    logic saw_mul_activity;
    logic saw_inorder_slot1_block;

    task automatic check(input string desc, input logic cond);
        if (!cond) begin
            $display("  FAIL: %s", desc);
            errors++;
        end else begin
            $display("  pass: %s", desc);
        end
    endtask

    task automatic clear_imem;
        for (int i = 0; i < 16; i++) imem[i] = NOP;
    endtask

    task automatic write_instr(input int idx, input logic [31:0] instr);
        imem[idx] = instr;
    endtask

    task automatic reset_dut;
        begin
            saw_mul_activity = 1'b0;
            saw_inorder_slot1_block = 1'b0;
            reset = 1'b1;
            repeat (3) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
        end
    endtask

    task automatic run_cycles(input int cycle_count);
        repeat (cycle_count) @(posedge clk);
    endtask

    task automatic load_test_basic_add_store_load;
        begin
            clear_imem();
            write_instr(0, 32'h00500093); // addi x1, x0, 5
            write_instr(1, 32'h00300113); // addi x2, x0, 3
            write_instr(2, 32'h002081B3); // add  x3, x1, x2
            write_instr(3, 32'h00302023); // sw   x3, 0(x0)
            write_instr(4, 32'h00002203); // lw   x4, 0(x0)
        end
    endtask

    task automatic load_test_mul_store_load;
        begin
            clear_imem();
            write_instr(0, 32'h00600093); // addi x1, x0, 6
            write_instr(1, 32'h00700113); // addi x2, x0, 7
            write_instr(2, 32'h022081B3); // mul  x3, x1, x2
            write_instr(3, 32'h00302223); // sw   x3, 4(x0)
            write_instr(4, 32'h00402203); // lw   x4, 4(x0)
        end
    endtask

    task automatic load_test_inorder_slot1_blocking;
        begin
            clear_imem();
            write_instr(0, 32'h00500093); // addi x1, x0, 5
            write_instr(1, 32'h00300113); // addi x2, x0, 3
            write_instr(2, 32'h002081B3); // add  x3, x1, x2  (will RAW stall)
            write_instr(3, 32'h00900293); // addi x5, x0, 9  (independent, must wait behind slot 0)
            write_instr(4, 32'h00302423); // sw   x3, 8(x0)
            write_instr(5, 32'h00502623); // sw   x5, 12(x0)
        end
    endtask

    always @(posedge clk) begin
        if (!reset) begin
            if (dut.raw_mul_done || dut.mul_done || dut.mul_pend_v)
                saw_mul_activity <= 1'b1;

            if ((dut.instr_queue0 == 32'h002081B3) &&
                (dut.instr_queue1 == 32'h00900293) &&
                dut.sb_stall0 && dut.sb_stall1 && (dut.disp_cnt == 2'd0))
                saw_inorder_slot1_block <= 1'b1;
        end
    end

    initial begin
        $display("\n=== Baseline Test Suite ===");

        // Test 1: existing add/store/load smoke test
        $display("\n[Test 1] add -> store -> load");
        load_test_basic_add_store_load();
        reset_dut();
        run_cycles(80);
        check("dmem[0] == 8 after add/store/load sequence",
              dmem[0] == 32'd8);
        check("PCF advanced past instruction 5 for add/store/load sequence",
              PCF >= 32'd24);

        // Test 2: multiplier path and memory writeback
        $display("\n[Test 2] mul -> store -> load");
        load_test_mul_store_load();
        reset_dut();
        run_cycles(100);
        check("multiplier path produced observable activity",
              saw_mul_activity);
        check("dmem[1] == 42 after mul/store/load sequence",
              dmem[1] == 32'd42);

        // Test 3: baseline in-order dispatch property
        $display("\n[Test 3] slot-1 blocked when slot-0 stalls");
        load_test_inorder_slot1_blocking();
        reset_dut();
        run_cycles(100);
        check("observed slot 1 blocked while slot 0 stalled at queue head",
              saw_inorder_slot1_block);
        check("dmem[2] == 8 after dependent add/store sequence",
              dmem[2] == 32'd8);
        check("dmem[3] == 9 after younger independent addi eventually retires",
              dmem[3] == 32'd9);

        // Summary
        $display("\n==============================");
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);
        $display("==============================\n");

        $finish;
    end

    // Safety timeout
    initial begin
        #10000;
        $display("TIMEOUT: simulation did not finish in time.");
        $finish;
    end

endmodule
