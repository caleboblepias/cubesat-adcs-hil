`timescale 1ns / 1ps

module plant_controller_tb;

    // ============================================================
    // DUT signals
    // ============================================================

    logic clk;
    logic rst;

    logic start_request;

    logic signed [31:0] wx;
    logic signed [31:0] wy;
    logic signed [31:0] wz;

    logic signed [31:0] torque_x;
    logic signed [31:0] torque_y;
    logic signed [31:0] torque_z;

    logic signed [31:0] dt;

    logic signed [31:0] ixx;
    logic signed [31:0] iyy;
    logic signed [31:0] izz;

    logic signed [31:0] ixx_inv;
    logic signed [31:0] iyy_inv;
    logic signed [31:0] izz_inv;

    logic busy;
    logic ready;

    logic state_update;

    logic signed [31:0] wx_next;
    logic signed [31:0] wy_next;
    logic signed [31:0] wz_next;

    integer errors;


    // ============================================================
    // DUT
    // ============================================================

    plant_controller dut (
        .clk           (clk),
        .rst           (rst),

        .start_request (start_request),

        .wx            (wx),
        .wy            (wy),
        .wz            (wz),

        .torque_x      (torque_x),
        .torque_y      (torque_y),
        .torque_z      (torque_z),

        .dt            (dt),

        .ixx           (ixx),
        .iyy           (iyy),
        .izz           (izz),

        .ixx_inv       (ixx_inv),
        .iyy_inv       (iyy_inv),
        .izz_inv       (izz_inv),

        .busy          (busy),
        .ready         (ready),

        .state_update  (state_update),

        .wx_next       (wx_next),
        .wy_next       (wy_next),
        .wz_next       (wz_next)
    );


    // ============================================================
    // 100 MHz clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end


    // ============================================================
    // Emulate persistent state registers
    //
    // This models the state_update behavior that we just verified
    // independently inside spi_registers.
    // ============================================================

    always @(posedge clk) begin

        if (state_update) begin

            wx <= wx_next;
            wy <= wy_next;
            wz <= wz_next;

        end

    end


    // ============================================================
    // Generate one-cycle START request
    // ============================================================

    task automatic pulse_start;

        begin

            @(negedge clk);
            start_request = 1'b1;

            @(negedge clk);
            start_request = 1'b0;

        end

    endtask


    // ============================================================
    // Wait for a completed state update
    // ============================================================

    task automatic wait_for_update;

        integer timeout;

        begin

            timeout = 0;

            while ((state_update !== 1'b1) &&
                   (timeout < 1000)) begin

                @(negedge clk);
                timeout = timeout + 1;

            end


            if (timeout >= 1000) begin

                $display(
                    "ERROR: timed out waiting for state_update"
                );

                errors = errors + 1;

            end

        end

    endtask


    // ============================================================
    // Check 32-bit value
    // ============================================================

    task automatic check_value (
        input logic signed [31:0] actual,
        input logic signed [31:0] expected,
        input string              name
    );

        begin

            if (actual !== expected) begin

                $display(
                    "ERROR %s: expected=%0d (0x%08h), actual=%0d (0x%08h)",
                    name,
                    expected,
                   expected,
                    actual,
                    actual
                );

                errors = errors + 1;

            end

        end

    endtask


    // ============================================================
    // Main tests
    // ============================================================

    initial begin

        integer errors_before;
        integer update_count;


        // ========================================================
        // Initialization
        // ========================================================

        errors = 0;

        rst           = 1'b1;
        start_request = 1'b0;

        wx = '0;
        wy = '0;
        wz = '0;

        torque_x = '0;
        torque_y = '0;
        torque_z = '0;

        dt = '0;

        ixx = '0;
        iyy = '0;
        izz = '0;

        ixx_inv = '0;
        iyy_inv = '0;
        izz_inv = '0;


        repeat (5) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;


        // ========================================================
        // Common spacecraft parameters
        //
        // Q16.16:
        //
        // dt      = 0.01
        // Ixx     = 0.02
        // Iyy     = 0.02
        // Izz     = 0.01
        // Ixx^-1  = 50
        // Iyy^-1  = 50
        // Izz^-1  = 100
        // ========================================================

        dt = 32'sd655;

        ixx = 32'sd1311;
        iyy = 32'sd1311;
        izz = 32'sd655;

        ixx_inv = 32'sd3276800;
        iyy_inv = 32'sd3276800;
        izz_inv = 32'sd6553600;


        // ========================================================
        // TEST 1: Reset status
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 1: RESET / IDLE STATUS");
        $display("-----------------------------------");

        errors_before = errors;

        if (busy !== 1'b0) begin
            $display("ERROR: busy not low after reset");
            errors = errors + 1;
        end

        if (ready !== 1'b0) begin
            $display("ERROR: ready not low after reset");
            errors = errors + 1;
        end

        if (state_update !== 1'b0) begin
            $display("ERROR: state_update high after reset");
            errors = errors + 1;
        end

        if (errors == errors_before)
            $display("TEST 1 PASSED");


        // ========================================================
        // TEST 2: One complete integration step
        //
        // Existing verified RK4 vector:
        //
        // W:
        //   6554, -13107, 19661
        //
        // torque:
        //   66, -98, 131
        //
        // expected:
        //   6567, -13166, 19791
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 2: SINGLE INTEGRATION STEP");
        $display("-----------------------------------");

        errors_before = errors;

        wx = 32'sd6554;
        wy = -32'sd13107;
        wz = 32'sd19661;

        torque_x = 32'sd66;
        torque_y = -32'sd98;
        torque_z = 32'sd131;


        pulse_start();


        // Controller should now be executing.
        if (busy !== 1'b1) begin
            $display("ERROR: busy did not assert");
            errors = errors + 1;
        end

        if (ready !== 1'b0) begin
            $display("ERROR: ready asserted during calculation");
            errors = errors + 1;
        end


        wait_for_update();


        check_value(
            wx_next,
            32'sd6567,
            "WX_NEXT"
        );

        check_value(
            wy_next,
            -32'sd13166,
            "WY_NEXT"
        );

        check_value(
            wz_next,
            32'sd19791,
            "WZ_NEXT"
        );


        // Allow emulated state registers to consume state_update.
        @(posedge clk);
        @(negedge clk);


        check_value(
            wx,
            32'sd6567,
            "committed WX"
        );

        check_value(
            wy,
            -32'sd13166,
            "committed WY"
        );

        check_value(
            wz,
            32'sd19791,
            "committed WZ"
        );


        if (busy !== 1'b0) begin
            $display("ERROR: busy not cleared after commit");
            errors = errors + 1;
        end

        if (ready !== 1'b1) begin
            $display("ERROR: ready not asserted after commit");
            errors = errors + 1;
        end


        if (errors == errors_before)
            $display("TEST 2 PASSED");


        // ========================================================
        // TEST 3: READY clears on next START
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 3: READY CLEARS ON START");
        $display("-----------------------------------");

        errors_before = errors;


        pulse_start();


        if (busy !== 1'b1) begin
            $display("ERROR: busy not asserted on second START");
            errors = errors + 1;
        end

        if (ready !== 1'b0) begin
            $display("ERROR: ready did not clear on second START");
            errors = errors + 1;
        end


        wait_for_update();

        @(posedge clk);
        @(negedge clk);


        if (ready !== 1'b1) begin
            $display("ERROR: ready not restored after second step");
            errors = errors + 1;
        end


        if (errors == errors_before)
            $display("TEST 3 PASSED");


        // ========================================================
        // TEST 4: START while BUSY is ignored
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 4: START WHILE BUSY");
        $display("-----------------------------------");

        errors_before = errors;
        update_count  = 0;


        pulse_start();


        if (busy !== 1'b1) begin
            $display("ERROR: controller did not enter BUSY");
            errors = errors + 1;
        end


        // Attempt another request while calculation is running.
        pulse_start();


        // Count state_update pulses until controller returns READY.
        while (ready !== 1'b1) begin

            @(negedge clk);

            if (state_update)
                update_count = update_count + 1;

        end


        if (update_count != 1) begin

            $display(
                "ERROR: expected one update, observed %0d",
                update_count
            );

            errors = errors + 1;

        end


        if (errors == errors_before)
            $display("TEST 4 PASSED");


        // ========================================================
        // TEST 5: state_update is exactly one clock
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 5: STATE UPDATE PULSE WIDTH");
        $display("-----------------------------------");

        errors_before = errors;


        pulse_start();

        wait_for_update();


        if (state_update !== 1'b1) begin
            $display("ERROR: state_update not asserted");
            errors = errors + 1;
        end


        @(posedge clk);
        @(negedge clk);


        if (state_update !== 1'b0) begin

            $display(
                "ERROR: state_update lasted more than one clock"
            );

            errors = errors + 1;

        end


        if (errors == errors_before)
            $display("TEST 5 PASSED");


        // ========================================================
        // TEST 6: Persistent multi-step plant state
        //
        // Reset state to the known initial vector, then execute
        // two consecutive steps without manually rewriting W.
        //
        // First step is the existing RK4 golden vector.
        // For the second step we primarily verify recursion:
        // W must change from the first result without software
        // rewriting the state.
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 6: CONSECUTIVE PLANT STEPS");
        $display("-----------------------------------");

        errors_before = errors;


        wx = 32'sd6554;
        wy = -32'sd13107;
        wz = 32'sd19661;

        torque_x = 32'sd66;
        torque_y = -32'sd98;
        torque_z = 32'sd131;


        // -------------------------
        // Step 1
        // -------------------------

        pulse_start();

        wait_for_update();

        check_value(
            wx_next,
            32'sd6567,
            "step 1 WX_NEXT"
        );

        check_value(
            wy_next,
            -32'sd13166,
            "step 1 WY_NEXT"
        );

        check_value(
            wz_next,
            32'sd19791,
            "step 1 WZ_NEXT"
        );


        @(posedge clk);
        @(negedge clk);


        check_value(
            wx,
            32'sd6567,
            "step 1 persistent WX"
        );

        check_value(
            wy,
            -32'sd13166,
            "step 1 persistent WY"
        );

        check_value(
            wz,
            32'sd19791,
            "step 1 persistent WZ"
        );


        // -------------------------
        // Step 2
        //
        // Do NOT rewrite wx/wy/wz.
        // The next RK4 operation must consume the committed result
        // from Step 1.
        // -------------------------

        pulse_start();

        wait_for_update();


        // If all three are still exactly the Step 1 result, the
        // controller clearly failed to propagate the plant.
        if ((wx_next === 32'sd6567) &&
            (wy_next === -32'sd13166) &&
            (wz_next === 32'sd19791)) begin

            $display(
                "ERROR: second integration step did not advance state"
            );

            errors = errors + 1;

        end


        @(posedge clk);
        @(negedge clk);


        // Persistent registers must equal the second RK4 result.
        check_value(
            wx,
            wx_next,
            "step 2 persistent WX"
        );

        check_value(
            wy,
            wy_next,
            "step 2 persistent WY"
        );

        check_value(
            wz,
            wz_next,
            "step 2 persistent WZ"
        );


        if (errors == errors_before)
            $display("TEST 6 PASSED");


        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("===================================");

        if (errors == 0) begin

            $display(
                "ALL PLANT CONTROLLER TESTS PASSED"
            );

        end else begin

            $display(
                "PLANT CONTROLLER TESTS FAILED: %0d errors",
                errors
            );

        end

        $display("===================================");
        $display("");

        $finish;

    end

endmodule 
