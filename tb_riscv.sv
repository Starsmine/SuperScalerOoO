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
            $display("         SB: stall0=%b stall1=%b | full A=%b M=%b Mem=%b",
                     dut.rs_stall0, dut.rs_stall1,
                     dut.adder_rs_full, dut.mul_rs_full, dut.mem_rs_full);
            $display("         RAT: tag_rs1_0=%0d tag_rs2_0=%0d tag_rs1_1=%0d tag_rs2_1=%0d",
                     dut.rat_tag_rs1_0, dut.rat_tag_rs2_0,
                     dut.rat_tag_rs1_1, dut.rat_tag_rs2_1);
            $display("         CDB: valid=%b tag=%0d val=%0d | issue A=%b M=%b Mem=%b",
                     dut.cdb_valid, dut.cdb_tag, dut.cdb_value,
                     dut.adder_issue_valid, dut.mul_issue_valid, dut.mem_issue_valid);
            $display("         TAG: raw_a=%b raw_m=%b raw_mem=%b | pend_a=%b pend_m=%b pend_mem=%b",
                     dut.raw_adder_done, dut.raw_mul_done, dut.raw_mem_done,
                     dut.u_cdb.adder_pend_v, dut.u_cdb.mul_pend_v, dut.u_cdb.mem_pend_v);
            $display("         WB: we=%b a3=%0d wd3=%0d | adder_wins=%b mul_wins=%b mem_wins=%b",
                     dut.ooo_reg_we, dut.ooo_reg_a3, dut.ooo_reg_wd3,
                     dut.u_cdb.adder_wins, dut.u_cdb.mul_wins, dut.u_cdb.mem_wins);
            $display("         MEM_FU: valid=%b addr=%08h wdata=%08h mwr=%b",
                     dut.mem_sc_valid[2], dut.sram_addr_r, dut.mem_sc_wdata[2], dut.mem_sc_memwrite[2]);
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
    logic saw_adder_rs_full;
    logic saw_queue_full;

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
            saw_adder_rs_full = 1'b0;
            saw_queue_full = 1'b0;
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

    // Test 4 — WAR (Write After Read)
    // add x3 reads x1=5, then addi x1 overwrites x1 with 99.
    // x3 must keep 8 (old x1), x1 must end up 99.
    task automatic load_test_war;
        begin
            clear_imem();
            write_instr(0, 32'h00500093); // addi x1, x0, 5
            write_instr(1, 32'h00300113); // addi x2, x0, 3
            write_instr(2, 32'h002081B3); // add  x3, x1, x2  -> x3 = 8 (reads x1)
            write_instr(3, 32'h06300093); // addi x1, x0, 99  -> x1 = 99 (WAR on x1)
            write_instr(4, 32'h00302023); // sw   x3, 0(x0)   -> dmem[0] = 8
            write_instr(5, 32'h00102223); // sw   x1, 4(x0)   -> dmem[1] = 99
        end
    endtask

    // Test 5 — WAW (Write After Write) across different-latency FUs
    // mul x3 (6-cy MUL) then addi x3 (4-cy ADDER): ADDER completes first
    // but addi is later in program order so its value (99) must win.
    // We read x3 well after both FUs have drained to catch stale
    // regfile overwrites.
    task automatic load_test_waw;
        begin
            clear_imem();
            write_instr(0,  32'h00500093); // addi x1, x0, 5
            write_instr(1,  32'h00700113); // addi x2, x0, 7
            write_instr(2,  32'h022081B3); // mul  x3, x1, x2  -> 35 (MUL, 6cy)
            write_instr(3,  32'h06300193); // addi x3, x0, 99  -> 99 (ADDER, 4cy, WAW)
            // 4-9: NOP (already cleared) — drain time for both FUs
            write_instr(10, 32'h00018233); // add  x4, x3, x0  -> x4 = x3 (late read)
            write_instr(11, 32'h00402023); // sw   x4, 0(x0)   -> dmem[0] should be 99
        end
    endtask

    // Test 6 — CDB arbitration contention
    // ADDER and MUL both produce results that will collide on CDB.
    // ADDER has higher priority; MUL result must pend one extra cycle.
    // Verify both values ultimately reach memory.
    task automatic load_test_cdb_contention;
        begin
            clear_imem();
            write_instr(0,  32'h00A00093); // addi x1, x0, 10
            write_instr(1,  32'h01400113); // addi x2, x0, 20
            write_instr(2,  32'h022081B3); // mul  x3, x1, x2  -> 200
            write_instr(3,  32'h04D00213); // addi x4, x0, 77  -> 77
            write_instr(4,  32'h00302023); // sw   x3, 0(x0)
            write_instr(5,  32'h00402223); // sw   x4, 4(x0)
        end
    endtask

    // Test 7 — Intra-pair RAW (slot-0 rd == slot-1 rs)
    // Slot 0: addi x1,x0,42 — writes x1
    // Slot 1: add  x2,x1,x0 — reads x1 (produced same-cycle by slot 0)
    // Tests the RAT slot-0→slot-1 bypass path.
    task automatic load_test_intra_pair_raw;
        begin
            clear_imem();
            write_instr(0, 32'h02A00093); // addi x1, x0, 42
            write_instr(1, 32'h00008133); // add  x2, x1, x0  -> x2 = x1 = 42
            write_instr(2, 32'h00202023); // sw   x2, 0(x0)   -> dmem[0] = 42
        end
    endtask

    // Test 8 — Dual dispatch to same FU
    // Both slots target ADDER simultaneously; exercises alloc1_ok.
    // Slot 0: addi x1, x0, 10  (ADDER)
    // Slot 1: addi x2, x0, 20  (ADDER, same FU)
    // Then store both.
    task automatic load_test_dual_dispatch_same_fu;
        begin
            clear_imem();
            write_instr(0, 32'h00A00093); // addi x1, x0, 10
            write_instr(1, 32'h01400113); // addi x2, x0, 20
            write_instr(2, 32'h00102023); // sw   x1, 0(x0)  -> dmem[0] = 10
            write_instr(3, 32'h00202223); // sw   x2, 4(x0)  -> dmem[1] = 20
        end
    endtask

    // Test 9 — All 3 FUs active simultaneously
    // ADDER, MUL, and MEM instructions all in flight at once.
    // addi x1=3, addi x2=4, mul x3=x1*x2=12, add x4=x1+x2=7, sw x1.
    // Then store results.
    task automatic load_test_all_3_fus;
        begin
            clear_imem();
            write_instr(0,  32'h00300093); // addi x1, x0, 3
            write_instr(1,  32'h00400113); // addi x2, x0, 4
            write_instr(2,  32'h022081B3); // mul  x3, x1, x2  -> 12 (MUL FU)
            write_instr(3,  32'h00208233); // add  x4, x1, x2  -> 7  (ADDER FU)
            write_instr(4,  32'h00102023); // sw   x1, 0(x0)   -> dmem[0] = 3 (MEM FU)
            write_instr(5,  32'h00302223); // sw   x3, 4(x0)   -> dmem[1] = 12
            write_instr(6,  32'h00402423); // sw   x4, 8(x0)   -> dmem[2] = 7
        end
    endtask

    // Test 10 — RS full-then-drain
    // Fill the ADDER RS (3 entries) with independent instructions,
    // verify they all drain correctly.
    // x1=1, x2=2, x3=3, then add x4=x1+x2, add x5=x2+x3, add x6=x1+x3
    // All 3 adds dispatch together filling ADDER RS, then drain.
    task automatic load_test_rs_full_drain;
        begin
            clear_imem();
            write_instr(0,  32'h00100093); // addi x1, x0, 1
            write_instr(1,  32'h00200113); // addi x2, x0, 2
            write_instr(2,  32'h00300193); // addi x3, x0, 3
            // After x1,x2,x3 are ready, dispatch 3 ADDs to fill ADDER RS
            write_instr(3,  32'h00208233); // add  x4, x1, x2  -> 3
            write_instr(4,  32'h003102B3); // add  x5, x2, x3  -> 5
            write_instr(5,  32'h00308333); // add  x6, x1, x3  -> 4
            write_instr(6,  32'h00402023); // sw   x4, 0(x0)   -> dmem[0] = 3
            write_instr(7,  32'h00502223); // sw   x5, 4(x0)   -> dmem[1] = 5
            write_instr(8,  32'h00602423); // sw   x6, 8(x0)   -> dmem[2] = 4
        end
    endtask

    // Test 11 — Queue back-pressure
    // 11 instructions: 1 seed + 9 RAW-chained addi x1 + 1 sw.
    // ADDER latency=4 means at most 1 dispatch/4 cycles while fetch
    // adds 2/cycle, causing the 8-deep queue to saturate.
    task automatic load_test_queue_backpressure;
        begin
            clear_imem();
            write_instr(0,  32'h00100093); // addi x1, x0, 1
            write_instr(1,  32'h00108093); // addi x1, x1, 1  -> x1=2 (RAW chain)
            write_instr(2,  32'h00108093); // addi x1, x1, 1  -> x1=3
            write_instr(3,  32'h00108093); // addi x1, x1, 1  -> x1=4
            write_instr(4,  32'h00108093); // addi x1, x1, 1  -> x1=5
            write_instr(5,  32'h00108093); // addi x1, x1, 1  -> x1=6
            write_instr(6,  32'h00108093); // addi x1, x1, 1  -> x1=7
            write_instr(7,  32'h00108093); // addi x1, x1, 1  -> x1=8
            write_instr(8,  32'h00108093); // addi x1, x1, 1  -> x1=9
            write_instr(9,  32'h00108093); // addi x1, x1, 1  -> x1=10
            write_instr(10, 32'h00102023); // sw   x1, 0(x0)   -> dmem[0] = 10
        end
    endtask

    // Test 12 — x0 destination (must remain zero)
    // addi x0, x0, 99 writes to x0; x0 must stay 0.
    // Then add x1 = x0 + x0 and store x1; dmem[0] must be 0.
    task automatic load_test_x0_dest;
        begin
            clear_imem();
            write_instr(0, 32'h06300013); // addi x0, x0, 99  -> x0 must stay 0
            write_instr(1, 32'h00000133); // add  x2, x0, x0  -> x2 = 0
            write_instr(2, 32'h00202023); // sw   x2, 0(x0)   -> dmem[0] = 0
        end
    endtask

    // Test 13 — Store-then-load same address
    // sw x1 to addr 0, then lw x2 from addr 0; x2 must get x1's value.
    task automatic load_test_store_then_load;
        begin
            clear_imem();
            write_instr(0, 32'h01B00093); // addi x1, x0, 27
            write_instr(1, 32'h00102023); // sw   x1, 0(x0)   -> dmem[0] = 27
            write_instr(2, 32'h00002103); // lw   x2, 0(x0)   -> x2 = 27
            write_instr(3, 32'h00202223); // sw   x2, 4(x0)   -> dmem[1] = 27
        end
    endtask

    always @(posedge clk) begin
        if (!reset) begin
            if (dut.raw_mul_done || dut.u_cdb.mul_wins || dut.u_cdb.mul_pend_v)
                saw_mul_activity <= 1'b1;

            if ((dut.instr_queue0 == 32'h002081B3) &&
                (dut.instr_queue1 == 32'h00900293) &&
                dut.rs_stall0 && dut.rs_stall1 && (dut.disp_cnt == 2'd0))
                saw_inorder_slot1_block <= 1'b1;

            if (dut.adder_rs_full)
                saw_adder_rs_full <= 1'b1;

            if (dut.instr_q.count >= 4'd7)
                saw_queue_full <= 1'b1;
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

        // Test 4: WAR hazard
        $display("\n[Test 4] WAR — overwrite source register after read");
        load_test_war();
        reset_dut();
        run_cycles(100);
        check("dmem[0] == 8 (add used old x1=5, not new x1=99)",
              dmem[0] == 32'd8);
        check("dmem[1] == 99 (new x1 stored correctly)",
              dmem[1] == 32'd99);

        // Test 5: WAW hazard across different-latency FUs
        $display("\n[Test 5] WAW — cross-FU write ordering");
        load_test_waw();
        reset_dut();
        run_cycles(120);
        check("dmem[0] == 99 (program-order-later addi must win over earlier mul)",
              dmem[0] == 32'd99);

        // Test 6: CDB contention — ADDER and MUL finish near-simultaneously
        $display("\n[Test 6] CDB contention — multi-FU collision");
        load_test_cdb_contention();
        reset_dut();
        run_cycles(120);
        check("dmem[0] == 200 (mul result survived CDB contention)",
              dmem[0] == 32'd200);
        check("dmem[1] == 77 (addi result survived CDB contention)",
              dmem[1] == 32'd77);

        // Test 7: Intra-pair RAW — slot 0 produces, slot 1 consumes same cycle
        $display("\n[Test 7] Intra-pair RAW — slot-0 to slot-1 bypass");
        load_test_intra_pair_raw();
        reset_dut();
        run_cycles(100);
        check("dmem[0] == 42 (slot 1 got slot 0's x1 via RAT bypass)",
              dmem[0] == 32'd42);

        // Test 8: Dual dispatch to same FU
        $display("\n[Test 8] Dual dispatch to same FU (both ADDER)");
        load_test_dual_dispatch_same_fu();
        reset_dut();
        run_cycles(80);
        check("dmem[0] == 10 (slot 0 addi x1=10)",
              dmem[0] == 32'd10);
        check("dmem[1] == 20 (slot 1 addi x2=20, same-FU dual dispatch)",
              dmem[1] == 32'd20);

        // Test 9: All 3 FUs active simultaneously
        $display("\n[Test 9] All 3 FUs active simultaneously");
        load_test_all_3_fus();
        reset_dut();
        run_cycles(120);
        check("dmem[0] == 3 (x1 stored via MEM while others in flight)",
              dmem[0] == 32'd3);
        check("dmem[1] == 12 (mul x3=3*4 via MUL FU)",
              dmem[1] == 32'd12);
        check("dmem[2] == 7 (add x4=3+4 via ADDER FU)",
              dmem[2] == 32'd7);

        // Test 10: RS full-then-drain
        $display("\n[Test 10] RS full-then-drain (3 ADDs fill ADDER RS)");
        load_test_rs_full_drain();
        reset_dut();
        run_cycles(120);
        check("saw ADDER RS full at some point",
              saw_adder_rs_full);
        check("dmem[0] == 3 (x4=x1+x2=1+2)",
              dmem[0] == 32'd3);
        check("dmem[1] == 5 (x5=x2+x3=2+3)",
              dmem[1] == 32'd5);
        check("dmem[2] == 4 (x6=x1+x3=1+3)",
              dmem[2] == 32'd4);

        // Test 11: Instruction queue back-pressure
        $display("\n[Test 11] Queue back-pressure (long RAW chain)");
        load_test_queue_backpressure();
        reset_dut();
        run_cycles(200);
        check("saw queue near-full (count>=7) at some point",
              saw_queue_full);
        check("dmem[0] == 10 (x1 incremented 1->10 through RAW chain)",
              dmem[0] == 32'd10);

        // Test 12: x0 destination
        $display("\n[Test 12] x0 destination must remain zero");
        load_test_x0_dest();
        reset_dut();
        run_cycles(80);
        check("dmem[0] == 0 (x0 stayed zero despite addi x0,x0,99)",
              dmem[0] == 32'd0);

        // Test 13: Store-then-load same address
        $display("\n[Test 13] Store-then-load same address");
        load_test_store_then_load();
        reset_dut();
        run_cycles(120);
        check("dmem[0] == 27 (store wrote 27 to addr 0)",
              dmem[0] == 32'd27);
        check("dmem[1] == 27 (load read 27 from addr 0, re-stored to addr 4)",
              dmem[1] == 32'd27);

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
        #20000;
        $display("TIMEOUT: simulation did not finish in time.");
        $finish;
    end

endmodule
