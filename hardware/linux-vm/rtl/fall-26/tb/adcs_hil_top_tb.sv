`timescale 1ns / 1ps

module adcs_hil_top_tb;

    // ============================================================
    // Register map
    // ============================================================

    localparam logic [7:0] CMD_WRITE = 8'h01;
    localparam logic [7:0] CMD_READ  = 8'h02;

    localparam logic [7:0] ADDR_STATUS  = 8'h00;
    localparam logic [7:0] ADDR_CONTROL = 8'h01;

    localparam logic [7:0] ADDR_TORQUE_X = 8'h10;
    localparam logic [7:0] ADDR_TORQUE_Y = 8'h14;
    localparam logic [7:0] ADDR_TORQUE_Z = 8'h18;

    localparam logic [7:0] ADDR_WX = 8'h20;
    localparam logic [7:0] ADDR_WY = 8'h24;
    localparam logic [7:0] ADDR_WZ = 8'h28;

    localparam logic [7:0] ADDR_DT = 8'h30;

    localparam logic [7:0] ADDR_IXX = 8'h40;
    localparam logic [7:0] ADDR_IYY = 8'h44;
    localparam logic [7:0] ADDR_IZZ = 8'h48;

    localparam logic [7:0] ADDR_IXX_INV = 8'h50;
    localparam logic [7:0] ADDR_IYY_INV = 8'h54;
    localparam logic [7:0] ADDR_IZZ_INV = 8'h58;


    // ============================================================
    // DUT signals
    // ============================================================

    logic clk;
    logic rst;

    logic spi_cs_n;
    logic spi_sclk;
    logic spi_mosi;
    logic spi_miso;

    integer errors;
    integer errors_before;


    // ============================================================
    // DUT
    // ============================================================

    adcs_hil_top dut (
        .clk      (clk),
        .rst      (rst),

        .spi_cs_n (spi_cs_n),
        .spi_sclk (spi_sclk),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );


    // ============================================================
    // 100 MHz FPGA clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end


    // ============================================================
    // SPI transaction helpers
    // ============================================================

    task automatic begin_transaction;
        begin

            spi_sclk = 1'b0;

            @(negedge clk);
            spi_cs_n = 1'b0;

            // Allow synchronized CS falling edge to propagate.
            repeat (5) @(posedge clk);

            #1;

        end
    endtask


    task automatic end_transaction;
        begin

            spi_sclk = 1'b0;

            @(negedge clk);
            spi_cs_n = 1'b1;

            // Allow transaction_end to propagate through the
            // synchronized SPI physical layer.
            repeat (5) @(posedge clk);

            #1;

        end
    endtask


    // ============================================================
    // Transfer one SPI Mode-0 byte
    // ============================================================

    task automatic spi_transfer_byte(
        input  logic [7:0] mosi_value,
        output logic [7:0] miso_value
    );

        integer bit_index;

        begin

            miso_value = 8'h00;

            for (
                bit_index = 7;
                bit_index >= 0;
                bit_index = bit_index - 1
            ) begin

                // MOSI setup while SCLK is low.
                spi_mosi = mosi_value[bit_index];

                #500;

                // Mode-0 rising sample edge.
                spi_sclk = 1'b1;

                #1;
                miso_value[bit_index] = spi_miso;

                #499;

                // Falling edge advances slave TX data.
                spi_sclk = 1'b0;

                #500;

            end

        end
    endtask


    // ============================================================
    // 32-bit register WRITE
    //
    // [01][ADDR][D31:24][D23:16][D15:8][D7:0]
    // ============================================================

    task automatic spi_write_reg(
        input logic [7:0]  address,
        input logic [31:0] value
    );

        logic [7:0] dummy;

        begin

            begin_transaction();

           spi_transfer_byte(CMD_WRITE, dummy);
            spi_transfer_byte(address,   dummy);

            spi_transfer_byte(value[31:24], dummy);
            spi_transfer_byte(value[23:16], dummy);
            spi_transfer_byte(value[15:8],  dummy);
            spi_transfer_byte(value[7:0],   dummy);

            end_transaction();

        end
    endtask


    // ============================================================
    // 32-bit register READ
    //
    // MOSI:
    // [02][ADDR][00][00][00][00]
    //
    // MISO:
    // [XX][XX][D31:24][D23:16][D15:8][D7:0]
    // ============================================================

    task automatic spi_read_reg(
        input  logic [7:0]  address,
        output logic [31:0] value
    );

        logic [7:0] dummy;
        logic [7:0] b3;
        logic [7:0] b2;
        logic [7:0] b1;
        logic [7:0] b0;

        begin

            begin_transaction();

            spi_transfer_byte(CMD_READ, dummy);
            spi_transfer_byte(address,  dummy);

            spi_transfer_byte(8'h00, b3);
            spi_transfer_byte(8'h00, b2);
            spi_transfer_byte(8'h00, b1);
            spi_transfer_byte(8'h00, b0);

            end_transaction();

            value = {
                b3,
                b2,
                b1,
                b0
            };

        end
    endtask


    // ============================================================
    // Register comparison helper
    // ============================================================

    task automatic expect_reg(
        input logic [7:0]  address,
        input logic [31:0] expected,
        input string       label
    );

        logic [31:0] actual;

        begin

            spi_read_reg(address, actual);

            if (actual !== expected) begin

                $display(
                    "ERROR %s: expected=%08h actual=%08h",
                    label,
                    expected,
                    actual
                );

                errors = errors + 1;

            end

        end
    endtask


    // ============================================================
    // Wait for READY through the actual SPI STATUS register
    // ============================================================

    task automatic wait_for_ready;

        logic [31:0] status;
        integer polls;

        begin

            status = 32'h00000000;
            polls  = 0;

            while (
                (status[1] !== 1'b1) &&
                (polls < 100)
            ) begin

                spi_read_reg(
                    ADDR_STATUS,
                    status
                );

                polls = polls + 1;

            end


            if (status[1] !== 1'b1) begin

                $display(
                    "ERROR: timeout waiting for READY, status=%08h",
                    status
                );

                errors = errors + 1;

            end

        end
    endtask


    // ============================================================
    // Main test sequence
    // ============================================================

    initial begin

        logic [31:0] status;
        logic [31:0] actual;

        logic [7:0] dummy;

        integer bit_index;


        // ========================================================
        // Initialization
        // ========================================================

        errors = 0;

        rst = 1'b1;

        spi_cs_n = 1'b1;
        spi_sclk = 1'b0;
        spi_mosi = 1'b0;

        repeat (10) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;

        repeat (10) @(posedge clk);


        // ========================================================
        // TEST 0: Reset STATUS
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 0: RESET STATUS");
        $display("-----------------------------------");

        errors_before = errors;

        spi_read_reg(
            ADDR_STATUS,
            status
        );

        // Reset semantics:
        // BUSY  = 0
        // READY = 0
        if (status !== 32'h00000000) begin

            $display(
                "ERROR reset STATUS: expected=00000000 actual=%08h",
                status
            );

            errors = errors + 1;

        end

        if (errors == errors_before)
            $display("TEST 0 PASSED");


        // ========================================================
        // TEST 1: Configure complete plant through SPI
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 1: SPI PLANT CONFIGURATION");
        $display("-----------------------------------");

        errors_before = errors;


        // dt = 0.01
        spi_write_reg(
            ADDR_DT,
            32'd655
        );


        // I = diag(0.02, 0.02, 0.01)
        spi_write_reg(
            ADDR_IXX,
            32'd1311
        );

        spi_write_reg(
            ADDR_IYY,
            32'd1311
        );

        spi_write_reg(
            ADDR_IZZ,
            32'd655
        );


        // I^-1 = diag(50, 50, 100)
        spi_write_reg(
            ADDR_IXX_INV,
            32'd3276800
        );

        spi_write_reg(
            ADDR_IYY_INV,
            32'd3276800
        );

        spi_write_reg(
            ADDR_IZZ_INV,
            32'd6553600
        );


        // Initial angular velocity:
        // (0.1, -0.2, 0.3) approximately in Q16.16.
        spi_write_reg(
            ADDR_WX,
            32'd6554
        );

        spi_write_reg(
            ADDR_WY,
            -32'sd13107
        );

        spi_write_reg(
            ADDR_WZ,
            32'd19661
        );


        // Torque vector.
        spi_write_reg(
            ADDR_TORQUE_X,
            32'd66
        );

        spi_write_reg(
            ADDR_TORQUE_Y,
            -32'sd98
        );

        spi_write_reg(
            ADDR_TORQUE_Z,
            32'd131
        );


        // Verify values came through the entire SPI stack.

        expect_reg(
            ADDR_DT,
            32'd655,
            "DT"
        );

        expect_reg(
            ADDR_IXX,
            32'd1311,
            "IXX"
        );

        expect_reg(
            ADDR_IYY,
            32'd1311,
            "IYY"
        );

        expect_reg(
            ADDR_IZZ,
            32'd655,
            "IZZ"
        );

        expect_reg(
            ADDR_IXX_INV,
            32'd3276800,
            "IXX_INV"
        );

        expect_reg(
            ADDR_IYY_INV,
            32'd3276800,
            "IYY_INV"
        );

        expect_reg(
            ADDR_IZZ_INV,
            32'd6553600,
            "IZZ_INV"
        );

        expect_reg(
            ADDR_WX,
            32'd6554,
            "initial WX"
        );

        expect_reg(
            ADDR_WY,
            -32'sd13107,
            "initial WY"
        );

        expect_reg(
            ADDR_WZ,
            32'd19661,
            "initial WZ"
        );

        expect_reg(
            ADDR_TORQUE_X,
            32'd66,
            "TORQUE_X"
        );

        expect_reg(
            ADDR_TORQUE_Y,
            -32'sd98,
            "TORQUE_Y"
        );

        expect_reg(
            ADDR_TORQUE_Z,
            32'd131,
            "TORQUE_Z"
        );


        if (errors == errors_before)
            $display("TEST 1 PASSED");


        // ========================================================
        // TEST 2: First complete plant step
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 2: FIRST PLANT STEP");
        $display("-----------------------------------");

        errors_before = errors;


        // CONTROL.START = 1.
        spi_write_reg(
            ADDR_CONTROL,
            32'h00000001
        );


        // Poll STATUS entirely through SPI.
        wait_for_ready();


        // Expected frozen RK4 result:
        //
        // (6554, -13107, 19661)
        //             ↓
        // (6567, -13166, 19791)

        expect_reg(
            ADDR_WX,
            32'd6567,
            "step 1 WX"
        );

        expect_reg(
            ADDR_WY,
            -32'sd13166,
            "step 1 WY"
        );

        expect_reg(
            ADDR_WZ,
            32'd19791,
            "step 1 WZ"
        );


        spi_read_reg(
            ADDR_STATUS,
            status
        );

        if (status !== 32'h00000002) begin

            $display(
                "ERROR step 1 STATUS: expected=00000002 actual=%08h",
                status
            );

            errors = errors + 1;

        end


        if (errors == errors_before)
            $display("TEST 2 PASSED");


        // ========================================================
        // TEST 3: Second step without rewriting W
        //
        // This proves WX/WY/WZ are persistent plant state.
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 3: PERSISTENT STATE");
        $display("-----------------------------------");

        errors_before = errors;


        // Do NOT rewrite WX/WY/WZ.
        // Simply start the plant again.

        spi_write_reg(
            ADDR_CONTROL,
            32'h00000001
        );

        wait_for_ready();


        // Exact second-step result from the frozen Q16.16 RK4:
        //
        // (6567, -13166, 19791)
        //             ↓
        // (6580, -13225, 19921)

        expect_reg(
            ADDR_WX,
            32'd6580,
            "step 2 WX"
        );

        expect_reg(
            ADDR_WY,
            -32'sd13225,
            "step 2 WY"
        );

        expect_reg(
            ADDR_WZ,
            32'd19921,
            "step 2 WZ"
        );


        if (errors == errors_before)
            $display("TEST 3 PASSED");


        // ========================================================
        // TEST 4: Explicit W write reinitializes plant state
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 4: STATE REINITIALIZATION");
        $display("-----------------------------------");

        errors_before = errors;


        spi_write_reg(
            ADDR_WX,
            32'd6554
        );

        spi_write_reg(
            ADDR_WY,
            -32'sd13107
        );

        spi_write_reg(
            ADDR_WZ,
            32'd19661
        );


        expect_reg(
            ADDR_WX,
            32'd6554,
            "reinit WX"
        );

        expect_reg(
            ADDR_WY,
            -32'sd13107,
            "reinit WY"
        );

        expect_reg(
            ADDR_WZ,
            32'd19661,
            "reinit WZ"
        );


        // Starting again from the explicit state must reproduce
        // the original first-step result.

        spi_write_reg(
            ADDR_CONTROL,
            32'h00000001
        );

        wait_for_ready();

        expect_reg(
            ADDR_WX,
            32'd6567,
            "reinit step WX"
        );

        expect_reg(
            ADDR_WY,
            -32'sd13166,
            "reinit step WY"
        );

        expect_reg(
            ADDR_WZ,
            32'd19791,
            "reinit step WZ"
        );


        if (errors == errors_before)
            $display("TEST 4 PASSED");


        // ========================================================
        // TEST 5: Abort partial SPI WRITE
        //
        // Start writing TORQUE_X but terminate CS before all four
        // data bytes arrive. The old value must remain intact.
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 5: PARTIAL WRITE ABORT");
        $display("-----------------------------------");

        errors_before = errors;


        // Establish known value.
        spi_write_reg(
            ADDR_TORQUE_X,
            32'h11223344
        );

        expect_reg(
            ADDR_TORQUE_X,
            32'h11223344,
            "pre-abort TORQUE_X"
        );


        // Begin:
        // WRITE, TORQUE_X, DE, AD
        //
        // Then terminate transaction before remaining bytes.

        begin_transaction();

        spi_transfer_byte(
            CMD_WRITE,
            dummy
        );

        spi_transfer_byte(
            ADDR_TORQUE_X,
            dummy
        );

        spi_transfer_byte(
            8'hDE,
            dummy
        );

        spi_transfer_byte(
            8'hAD,
            dummy
        );

        end_transaction();


        // Partial transaction must not have committed.

        expect_reg(
            ADDR_TORQUE_X,
            32'h11223344,
            "post-abort preserved TORQUE_X"
        );


        // A clean transaction immediately afterward must work.

        spi_write_reg(
            ADDR_TORQUE_X,
            32'hCAFEBABE
        );

        expect_reg(
            ADDR_TORQUE_X,
            32'hCAFEBABE,
            "post-abort clean TORQUE_X"
        );


        if (errors == errors_before)
            $display("TEST 5 PASSED");


        // ========================================================
        // TEST 6: Invalid register read
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 6: INVALID REGISTER READ");
        $display("-----------------------------------");

        errors_before = errors;


        spi_read_reg(
            8'hFF,
            actual
        );

        if (actual !== 32'h00000000) begin

            $display(
                "ERROR invalid read: expected=00000000 actual=%08h",
                actual
            );

            errors = errors + 1;

        end


        if (errors == errors_before)
            $display("TEST 6 PASSED");


        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("===================================");

        if (errors == 0) begin

            $display(
                "ALL ADCS HIL INTEGRATION TESTS PASSED"
            );

        end else begin

            $display(
                "ADCS HIL INTEGRATION TESTS FAILED: %0d errors",
                errors
            );

        end

        $display("===================================");
        $display("");

        $finish;

    end

endmodule 
