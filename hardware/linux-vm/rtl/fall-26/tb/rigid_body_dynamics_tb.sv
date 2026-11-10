`timescale 1ns / 1ps

module rigid_body_dynamics_tb;

    logic clk;
    logic rst;
    logic start;
    logic done;

    // DUT inputs - signed Q16.16
    logic signed [31:0] wx;
    logic signed [31:0] wy;
    logic signed [31:0] wz;

    logic signed [31:0] torque_x;
    logic signed [31:0] torque_y;
    logic signed [31:0] torque_z;

    logic signed [31:0] Ixx;
    logic signed [31:0] Iyy;
    logic signed [31:0] Izz;

    logic signed [31:0] Ixx_inv;
    logic signed [31:0] Iyy_inv;
    logic signed [31:0] Izz_inv;

    // DUT outputs - signed Q16.16
    logic signed [31:0] wx_dot;
    logic signed [31:0] wy_dot;
    logic signed [31:0] wz_dot;


    // ============================================================
    // DUT
    // ============================================================

    rigid_body_dynamics dut (
        .clk      (clk),
        .rst      (rst),
        .start    (start),

        .wx       (wx),
        .wy       (wy),
        .wz       (wz),

        .torque_x (torque_x),
        .torque_y (torque_y),
        .torque_z (torque_z),

        .Ixx      (Ixx),
        .Iyy      (Iyy),
        .Izz      (Izz),

        .Ixx_inv  (Ixx_inv),
        .Iyy_inv  (Iyy_inv),
        .Izz_inv  (Izz_inv),

        .done     (done),

        .wx_dot   (wx_dot),
        .wy_dot   (wy_dot),
        .wz_dot   (wz_dot)
    );


    // ============================================================
    // 100 MHz clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end


    // ============================================================
    // Bit-exact numerical test
    // ============================================================

    task automatic run_fixed_test(
        input integer test_number,

        input logic signed [31:0] test_wx,
        input logic signed [31:0] test_wy,
        input logic signed [31:0] test_wz,

        input logic signed [31:0] test_tx,
        input logic signed [31:0] test_ty,
        input logic signed [31:0] test_tz,

        input logic signed [31:0] expected_wx_dot,
        input logic signed [31:0] expected_wy_dot,
        input logic signed [31:0] expected_wz_dot
    );

        begin

            // Apply inputs away from the DUT sampling edge.
            @(negedge clk);

            wx       = test_wx;
            wy       = test_wy;
            wz       = test_wz;

            torque_x = test_tx;
            torque_y = test_ty;
            torque_z = test_tz;


            // Assert start before the next positive edge.
            start = 1'b1;

            // DUT samples inputs here.
            @(posedge clk);

            // Deassert start away from sampling edge.
            @(negedge clk);
            start = 1'b0;


            // Wait for result.
            @(posedge done);


            $display("");
            $display("-----------------------------------");
            $display("TEST %0d", test_number);
            $display("-----------------------------------");

            $display(
                "wx_dot: expected=%0d actual=%0d",
                expected_wx_dot,
                wx_dot
            );

            $display(
                "wy_dot: expected=%0d actual=%0d",
                expected_wy_dot,
                wy_dot
            );

            $display(
                "wz_dot: expected=%0d actual=%0d",
                expected_wz_dot,
                wz_dot
            );


            if (wx_dot !== expected_wx_dot) begin
                $error(
                    "TEST %0d FAILED: wx_dot mismatch",
                    test_number
                );
                $fatal;
            end

            if (wy_dot !== expected_wy_dot) begin
                $error(
                    "TEST %0d FAILED: wy_dot mismatch",
                    test_number
                );
                $fatal;
            end

            if (wz_dot !== expected_wz_dot) begin
                $error(
                    "TEST %0d FAILED: wz_dot misatch",
                    test_number
                );
                $fatal;
            end


            $display("TEST %0d PASSED", test_number);


            // Wait until DUT has returned to IDLE.
            @(posedge clk);
            @(negedge clk);

        end

    endtask


    // ============================================================
    // Input-latching and done-pulse test
    // ============================================================

    task automatic test_input_latching;

        begin

            $display("");
            $display("-----------------------------------");
            $display("TEST 5: INPUT LATCHING / HANDSHAKE");
            $display("-----------------------------------");


            // ----------------------------------------------------
            // Apply known Test 3 inputs away from positive edge
            // ----------------------------------------------------

            @(negedge clk);

            wx       =  32'sd6554;
            wy       = -32'sd13107;
            wz       =  32'sd19661;

            torque_x =  32'sd66;
            torque_y = -32'sd98;
            torque_z =  32'sd131;

            start = 1'b1;


            // ----------------------------------------------------
            // DUT samples start and all inputs on this edge
            // ----------------------------------------------------

            @(posedge clk);


            // ----------------------------------------------------
            // Wait until negative edge before changing anything.
            //
            // This avoids a simulation race with the DUT's
            // posedge-triggered always_ff block.
            // ----------------------------------------------------

            @(negedge clk);

            start = 1'b0;


            // Corrupt external inputs after DUT has latched them.
            wx       =  32'sd123456;
            wy       =  32'sd234567;
            wz       = -32'sd345678;

            torque_x = -32'sd1000;
            torque_y =  32'sd2000;
            torque_z = -32'sd3000;


            // ----------------------------------------------------
            // Wait for original calculation
            // ----------------------------------------------------

            @(posedge done);


            // ----------------------------------------------------
            // Original inputs must have been used
            // ----------------------------------------------------

            if (wx_dot !== 32'sd1350) begin
                $error(
                    "Input latching failed: wx_dot expected=1350 actual=%0d",
                    wx_dot
                );
                $fatal;
            end

            if (wy_dot !== -32'sd5850) begin
                $error(
                    "Input latching failed: wy_dot expected=-5850 actual=%0d",
                    wy_dot
                );
                $fatal;
            end

            if (wz_dot !== 32'sd13100) begin
                $error(
                    "Input latching failed: wz_dot expected=13100 actual=%0d",
                    wz_dot
                );
                $fatal;
            end


            // done must currently be asserted.
            if (done !== 1'b1) begin
                $error("done was not asserted");
                $fatal;
            end


            // ----------------------------------------------------
            // Verify done is exactly one clock cycle
            // ----------------------------------------------------

            @(posedge clk);

            // Let nonblocking assignments settle.
            #1;

            if (done !== 1'b0) begin
                $error(
                    "done remained asserted for more than one clock cycle"
                );
                $fatal;
            end


            $display("Input values successfully latched");
            $display("done pulse is exactly one clock cycle");
            $display("TEST 5 PASSED");

        end

    endtask


    // ============================================================
    // Main test sequence
    // ============================================================

    initial begin

        rst   = 1'b1;
        start = 1'b0;

        wx = '0;
        wy = '0;
        wz = '0;

        torque_x = '0;
        torque_y = '0;
        torque_z = '0;


        // --------------------------------------------------------
        // Spacecraft inertia constants
        //
        // Q16.16:
        //
        // 0.02  = 1311
        // 0.01  = 655
        // 50.0  = 3276800
        // 100.0 = 6553600
        // --------------------------------------------------------

        Ixx = 32'sd1311;
        Iyy = 32'sd1311;
        Izz = 32'sd655;

        Ixx_inv = 32'sd3276800;
        Iyy_inv = 32'sd3276800;
        Izz_inv = 32'sd6553600;


        // --------------------------------------------------------
        // Reset
        // --------------------------------------------------------

        repeat (5) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;


        // ========================================================
        // TEST 0
        // Zero state, zero torque
        // ========================================================

        run_fixed_test(
            0,

            32'sd0,
            32'sd0,
            32'sd0,

            32'sd0,
            32'sd0,
            32'sd0,

            32'sd0,
            32'sd0,
            32'sd0
        );


        // ========================================================
        // TEST 1
        // Torque only
        // ========================================================

        run_fixed_test(
            1,

             32'sd0,
             32'sd0,
             32'sd0,

             32'sd66,
            -32'sd98,
             32'sd131,

             32'sd3300,
            -32'sd4900,
             32'sd13100
        );


        // ========================================================
        // TEST 2
        // Rotational coupling only
        // ========================================================

        run_fixed_test(
            2,

             32'sd6554,
            -32'sd13107,
             32'sd19661,

             32'sd0,
             32'sd0,
             32'sd0,

            -32'sd1950,
            -32'sd950,
             32'sd0
        );


        // ========================================================
        // TEST 3
        // Full rigid-body dynamics
        // ========================================================

        run_fixed_test(
            3,

             32'sd6554,
            -32'sd13107,
             32'sd19661,

             32'sd66,
            -32'sd98,
             32'sd131,

             32'sd1350,
            -32'sd5850,
             32'sd13100
        );


        // ========================================================
        // TEST 4
        // Mixed signs
        // ========================================================

        run_fixed_test(
            4,

            -32'sd26214,
             32'sd9830,
            -32'sd16384,

            -32'sd66,
             32'sd33,
            -32'sd98,

            -32'sd4500,
            -32'sd1600,
            -32'sd9800
        );


        // ========================================================
        // TEST 5
        // Input latching + one-cycle done pulse
        // ========================================================

        test_input_latching;


        // ========================================================
        // Success
        // ========================================================

        $display("");
        $display("===================================");
        $display("ALL TESTS PASSED");
        $display("===================================");

        $finish;

    end

endmodule
