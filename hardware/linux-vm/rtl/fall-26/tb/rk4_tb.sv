`timescale 1ns / 1ps

module rk4_tb;

    logic clk;
    logic rst;
    logic start;
    logic done;

    // Initial angular velocity, Q16.16
    logic signed [31:0] wx;
    logic signed [31:0] wy;
    logic signed [31:0] wz;

    // Applied torque, Q16.16
    logic signed [31:0] torque_x;
    logic signed [31:0] torque_y;
    logic signed [31:0] torque_z;

    // Integration timestep, Q16.16
    logic signed [31:0] dt;

    // Inertia, Q16.16
    logic signed [31:0] Ixx;
    logic signed [31:0] Iyy;
    logic signed [31:0] Izz;

    logic signed [31:0] Ixx_inv;
    logic signed [31:0] Iyy_inv;
    logic signed [31:0] Izz_inv;

    // Integrated angular velocity
    logic signed [31:0] wx_next;
    logic signed [31:0] wy_next;
    logic signed [31:0] wz_next;


    // ============================================================
    // DUT
    // ============================================================

    rk4 dut (
        .clk      (clk),
        .rst      (rst),

        .start    (start),
        .done     (done),

        .wx       (wx),
        .wy       (wy),
        .wz       (wz),

        .torque_x (torque_x),
        .torque_y (torque_y),
        .torque_z (torque_z),

        .dt       (dt),

        .Ixx      (Ixx),
        .Iyy      (Iyy),
        .Izz      (Izz),

        .Ixx_inv  (Ixx_inv),
        .Iyy_inv  (Iyy_inv),
        .Izz_inv  (Izz_inv),

        .wx_next  (wx_next),
        .wy_next  (wy_next),
        .wz_next  (wz_next)
    );


    // ============================================================
    // 100 MHz clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end


    // ============================================================
    // Run one RK4 transaction
    // ============================================================

    task automatic run_test(
        input integer test_number,

        input logic signed [31:0] test_wx,
        input logic signed [31:0] test_wy,
        input logic signed [31:0] test_wz,

        input logic signed [31:0] test_tx,
        input logic signed [31:0] test_ty,
        input logic signed [31:0] test_tz,

        input logic signed [31:0] expected_wx,
        input logic signed [31:0] expected_wy,
        input logic signed [31:0] expected_wz
    );

        begin

            // ----------------------------------------------------
            // Apply transaction away from positive clock edge
            // ----------------------------------------------------

            @(negedge clk);

            wx = test_wx;
            wy = test_wy;
            wz = test_wz;

            torque_x = test_tx;
            torque_y = test_ty;
            torque_z = test_tz;

            start = 1'b1;


            // DUT captures transaction here.
            @(posedge clk);


            // Deassert start away from sampling edge.
            @(negedge clk);
            start = 1'b0;


            // ----------------------------------------------------
            // Wait for complete RK4 integration
            // ----------------------------------------------------

            @(posedge done);


            // ----------------------------------------------------
            // Display result
            // ----------------------------------------------------

            $display("");
            $display("-----------------------------------");
            $display("TEST %0d", test_number);
            $display("-----------------------------------");

            $display(
                "wx_next: expected=%0d actual=%0d",
                expected_wx,
                wx_next
            );

            $display(
                "wy_next: expected=%0d actual=%0d",
                expected_wy,
                wy_next
            );

            $display(
                "wz_next: expected=%0d actual=%0d",
                expected_wz,
                wz_next
            );


            // ----------------------------------------------------
           // Exact bit-for-bit comparisons
            // ----------------------------------------------------

            if (wx_next !== expected_wx) begin
                $error(
                    "TEST %0d FAILED: wx_next mismatch",
                    test_number
                );
                $fatal;
            end

            if (wy_next !== expected_wy) begin
                $error(
                    "TEST %0d FAILED: wy_next mismatch",
                    test_number
                );
                $fatal;
            end

            if (wz_next !== expected_wz) begin
                $error(
                    "TEST %0d FAILED: wz_next mismatch",
                    test_number
                );
                $fatal;
            end


            $display("TEST %0d PASSED", test_number);


            // Wait for DONE -> IDLE.
            @(posedge clk);
            @(negedge clk);

        end

    endtask


    // ============================================================
    // Input-latching / handshake test
    // ============================================================

    task automatic test_input_latching;

        begin

            $display("");
            $display("-----------------------------------");
            $display("TEST 5: INPUT LATCHING / HANDSHAKE");
            $display("-----------------------------------");


            // ----------------------------------------------------
            // Apply Test 3 transaction
            // ----------------------------------------------------

            @(negedge clk);

            wx =  32'sd6554;
            wy = -32'sd13107;
            wz =  32'sd19661;

            torque_x =  32'sd66;
            torque_y = -32'sd98;
            torque_z =  32'sd131;

            start = 1'b1;


            // DUT captures everything here.
            @(posedge clk);


            // ----------------------------------------------------
            // Corrupt every external transaction input after
            // capture. The active RK4 integration must be
            // completely unaffected.
            // ----------------------------------------------------

            @(negedge clk);

            start = 1'b0;

            wx =  32'sd123456;
            wy = -32'sd234567;
            wz =  32'sd345678;

            torque_x = -32'sd5000;
            torque_y =  32'sd6000;
            torque_z = -32'sd7000;

            dt = 32'sd12345;

            Ixx = 32'sd999;
            Iyy = 32'sd888;
            Izz = 32'sd777;

            Ixx_inv = 32'sd111111;
            Iyy_inv = 32'sd222222;
            Izz_inv = 32'sd333333;


            // ----------------------------------------------------
            // Wait for completion
            // ----------------------------------------------------

            @(posedge done);


            // Test 3 golden output must still appear.
            if (wx_next !== 32'sd6567) begin
                $error(
                    "Input latching failed: wx_next expected=6567 actual=%0d",
                    wx_next
                );
                $fatal;
            end

            if (wy_next !== -32'sd13166) begin
                $error(
                    "Input latching failed: wy_next expected=-13166 actual=%0d",
                    wy_next
                );
                $fatal;
            end

            if (wz_next !== 32'sd19791) begin
                $error(
                    "Input latching failed: wz_next expected=19791 actual=%0d",
                    wz_next
                );
                $fatal;
            end


            // done should currently be high.
            if (done !== 1'b1) begin
                $error("done was not asserted");
                $fatal;
            end


            // ----------------------------------------------------
            // done must be a one-clock pulse
            // ----------------------------------------------------

            @(posedge clk);
            #1;

            if (done !== 1'b0) begin
                $error(
                    "done remained asserted for more than one cycle"
                );
                $fatal;
            end


            $display("RK4 transaction inputs successfully latched");
            $display("done pulse is exactly one clock cycle");
            $display("TEST 5 PASSED");

        end

    endtask


    // ============================================================
    // Main test sequence
    // ============================================================

    initial begin

        rst = 1'b1;
        start = 1'b0;

        wx = '0;
        wy = '0;
        wz = '0;

        torque_x = '0;
        torque_y = '0;
        torque_z = '0;


        // --------------------------------------------------------
        // Golden constants from Python Q16.16 model
        // --------------------------------------------------------

        dt = 32'sd655;

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
        // TEST 0: Zero
        //
        // Python:
        // next = (0, 0, 0)
        // ========================================================

        run_test(
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
        // TEST 1: Torque only
        //
        // Python:
        // next = (32, -49, 130)
        // ========================================================

        run_test(
            1,

            32'sd0,
            32'sd0,
            32'sd0,

             32'sd66,
            -32'sd98,
             32'sd131,

             32'sd32,
            -32'sd49,
             32'sd130
        );


        // ========================================================
        // TEST 2: Rotational coupling only
        //
        // Python:
        // next = (6534, -13117, 19661)
        // ========================================================

        run_test(
            2,

             32'sd6554,
            -32'sd13107,
             32'sd19661,

             32'sd0,
             32'sd0,
             32'sd0,

             32'sd6534,
            -32'sd13117,
             32'sd19661
        );


        // ========================================================
        // TEST 3: Full dynamics
        //
        // Python:
        // next = (6567, -13166, 19791)
        // ========================================================

        run_test(
            3,

             32'sd6554,
            -32'sd13107,
             32'sd19661,

             32'sd66,
            -32'sd98,
             32'sd131,

             32'sd6567,
            -32'sd13166,
             32'sd19791
        );


        // ========================================================
        // TEST 4: Mixed signs
        //
        // Python:
        // next = (-26259, 9813, -16482)
        //
        // This is especially useful because K4_y differs from
        // K1/K2/K3, exercising the RK4 sequencing.
        // ========================================================

        run_test(
            4,

            -32'sd26214,
             32'sd9830,
            -32'sd16384,

            -32'sd66,
             32'sd33,
            -32'sd98,

            -32'sd26259,
             32'sd9813,
            -32'sd16482
        );


        // ========================================================
        // TEST 5: Input latching + done pulse
        // ========================================================

        test_input_latching;


        // ========================================================
        // Success
        // ========================================================

        $display("");
        $display("===================================");
        $display("ALL RK4 TESTS PASSED");
        $display("===================================");

        $finish;

    end

endmodule 
