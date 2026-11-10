`timescale 1ns / 1ps

module qmul_sequential_tb;

    logic clk;
    logic rst;

    logic start;
    logic done;

    logic signed [31:0] a;
    logic signed [31:0] b;
    logic signed [31:0] result;


    // ============================================================
    // DUT
    // ============================================================

    qmul_sequential dut (
        .clk    (clk),
        .rst    (rst),
        .start  (start),
        .done   (done),
        .a      (a),
        .b      (b),
        .result (result)
    );


    // ============================================================
    // Clock
    // 125 MHz = 8 ns period
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #4 clk = ~clk;
    end


    // ============================================================
    // Reference qmul
    //
    // This is exactly the arithmetic implemented by the old
    // combinational qmul() function.
    // ============================================================

    function automatic logic signed [31:0] reference_qmul;
        input logic signed [31:0] ref_a;
        input logic signed [31:0] ref_b;

        logic signed [63:0] product;

        begin
            product = ref_a * ref_b;
            reference_qmul = product >>> 16;
        end
    endfunction


    // ============================================================
    // Test helper
    // ============================================================

    task automatic run_test;
        input logic signed [31:0] test_a;
        input logic signed [31:0] test_b;

        logic signed [31:0] expected;

        begin

            expected = reference_qmul(test_a, test_b);

            // Present operands.
            @(negedge clk);

            a = test_a;
            b = test_b;
            start = 1'b1;

            // One-cycle start pulse.
            @(negedge clk);

            start = 1'b0;

            // Wait for sequential multiplier.
            wait (done == 1'b1);

            // Check result while done is asserted.
            if (result !== expected) begin

                $display(
                    "FAIL: a=%0d b=%0d expected=%0d got=%0d",
                    test_a,
                    test_b,
                    expected,
                    result
                );

                $fatal;

            end else begin

                $display(
                    "PASS: a=%0d b=%0d result=%0d",
                    test_a,
                    test_b,
                    result
                );

            end

            // Ensure done is a one-cycle pulse.
            @(posedge clk);
            #1;

            if (done !== 1'b0) begin
                $display("FAIL: done did not deassert");
                $fatal;
            end

        end
    endtask


    // ============================================================
    // Tests
    // ============================================================

    initial begin

        rst   = 1'b1;
        start = 1'b0;

        a = '0;
        b = '0;

        repeat (4) @(posedge clk);

        rst = 1'b0;

        repeat (2) @(posedge clk);


        // --------------------------------------------------------
        // Basic zero cases
        // --------------------------------------------------------

        run_test(
            32'sd0,
            32'sd65536
        );

        run_test(
            32'sd65536,
            32'sd0
        );


        // --------------------------------------------------------
        // +/- 1.0
        //
        // 1.0  = 65536
        // -1.0 = -65536
        // --------------------------------------------------------

        run_test(
            32'sd65536,
            32'sd65536
        );

        run_test(
            32'sd65536,
            -32'sd65536
        );

        run_test(
            -32'sd65536,
            32'sd65536
        );

        run_test(
            -32'sd65536,
            -32'sd65536
        );


        // -------------------------------------------------------
        // Representative ADCS values
        // --------------------------------------------------------

        // dt = 0.01
        // angular acceleration ~= 0.05035
        run_test(
            32'sd655,
            32'sd3300
        );

        // dt = 0.01
        // negative angular acceleration
        run_test(
            32'sd655,
            -32'sd4900
        );

        // I^-1 = 50.0
        // torque = 0.002
        run_test(
            32'sd3276800,
            32'sd131
        );

        // I^-1 = 100.0
        // torque = 0.002
        run_test(
            32'sd6553600,
            32'sd131
        );


        // --------------------------------------------------------
        // Arbitrary values to exercise discarded fractional bits
        // --------------------------------------------------------

        run_test(
            32'sd12345,
            32'sd6789
        );

        run_test(
            -32'sd12345,
            32'sd6789
        );

        run_test(
            32'sd12345,
            -32'sd6789
        );

        run_test(
            -32'sd12345,
            -32'sd6789
        );


        // --------------------------------------------------------
        // Larger values
        // --------------------------------------------------------

        run_test(
            32'sd131072,
            32'sd196608
        );

        run_test(
            -32'sd131072,
            32'sd196608
        );


        // --------------------------------------------------------
        // Signed edge cases
        //
        // These are useful for validating the magnitude conversion,
        // particularly INT_MIN.
        // --------------------------------------------------------

        run_test(
            32'sh80000000,
            32'sd1
        );

        run_test(
            32'sh80000000,
            -32'sd1
        );

        run_test(
            32'sh80000000,
            32'sh00010000
        );


        $display("");
        $display("========================================");
        $display("ALL QMUL SEQUENTIAL TESTS PASSED");
        $display("========================================");
        $display("");

        $finish;

    end

endmodule
