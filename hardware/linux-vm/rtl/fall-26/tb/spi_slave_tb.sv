`timescale 1ns / 1ps

module spi_slave_tb;

    logic clk;
    logic rst;

    logic spi_cs_n;
    logic spi_sclk;
    logic spi_mosi;
    logic spi_miso;

    logic [7:0] rx_byte;
    logic       rx_valid;

    logic [7:0] tx_byte;
    logic       tx_load;

    logic transaction_end;

    integer errors;
    integer errors_before;


    // ============================================================
    // DUT
    // ============================================================

    spi_slave dut (
        .clk             (clk),
        .rst             (rst),

        .spi_cs_n        (spi_cs_n),
        .spi_sclk        (spi_sclk),
        .spi_mosi        (spi_mosi),
        .spi_miso        (spi_miso),

        .rx_byte         (rx_byte),
        .rx_valid        (rx_valid),

        .tx_byte         (tx_byte),
        .tx_load         (tx_load),

        .transaction_end (transaction_end)
    );


    // ============================================================
    // 100 MHz FPGA clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end


    // ============================================================
    // Load TX byte
    // ============================================================

    task automatic load_tx_byte(
        input logic [7:0] value
    );
        begin
            @(negedge clk);

            tx_byte = value;
            tx_load = 1'b1;

            @(negedge clk);

            tx_load = 1'b0;

            #1;
        end
    endtask


    // ============================================================
    // Begin transaction
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


    // ============================================================
    // End transaction
    // ============================================================

    task automatic end_transaction;
        begin
            spi_sclk = 1'b0;

            @(negedge clk);
            spi_cs_n = 1'b1;

            // Allow synchronized CS rising edge to propagate.
            repeat (5) @(posedge clk);

            #1;
        end
    endtask


    // ============================================================
    // SPI Mode-0 byte transfer
    //
    // MOSI changes while SCLK is low.
    // Master samples MISO on rising SCLK.
    //
    // rx_valid is monitored concurrently because it is only one
    // FPGA clock wide.
    // ============================================================

    task automatic transfer_byte(
        input  logic [7:0] mosi_value,
        output logic [7:0] miso_value,
        output logic [7:0] received_value,
        output integer     rx_pulse_count
    );

        integer bit_index;
        integer monitor_cycles;

        begin

            miso_value     = 8'h00;
            received_value = 8'h00;
            rx_pulse_count = 0;

            fork

                // -----------------------------------------------
                // SPI master
                // -----------------------------------------------

                begin

                    for (
                        bit_index = 7;
                        bit_index >= 0;
                        bit_index = bit_index - 1
                    ) begin

                        // MOSI setup during low phase.
                        spi_mosi = mosi_value[bit_index];

                        #500;

                        // Rising edge: Mode-0 sampling edge.
                        spi_sclk = 1'b1;

                        // Sample essentially immediately after the
                        // external rising edge. The DUT synchronizer
                        // has not shifted its TX register yet.
                        #1;
                        miso_value[bit_index] = spi_miso;

                        #499;

                        // Falling edge: slave advances TX data.
                        spi_sclk = 1'b0;

                        #500;

                    end

                end


                // -----------------------------------------------
                // Monitor FPGA-domain rx_valid
                // -----------------------------------------------

                begin

                    monitor_cycles = 0;

                    while (monitor_cycles < 1300) begin

                        @(negedge clk);

                        if (rx_valid) begin
                            rx_pulse_count = rx_pulse_count + 1;
                            received_value = rx_byte;
                        end

                        monitor_cycles = monitor_cycles + 1;

                    end

                end

            join

        end

    endtask


    // ============================================================
    // Check one normal byte transfer
    // ============================================================

    task automatic check_transfer(
        input logic [7:0] mosi_value,
        input logic [7:0] expected_miso,
        input string      label
    );

        logic [7:0] master_rx;
        logic [7:0] slave_rx;
        integer     rx_count;

        begin

            transfer_byte(
                mosi_value,
                master_rx,
                slave_rx,
                rx_count
            );

            if (master_rx !== expected_miso) begin

                $display(
                    "ERROR %s TX: expected=%02h actual=%02h",
                    label,
                    expected_miso,
                    master_rx
                );

                errors = errors + 1;

            end

            if (rx_count != 1) begin

                $display(
                    "ERROR %s rx_valid count: expected=1 actual=%0d",
                    label,
                    rx_count
                );

                errors = errors + 1;

            end else if (slave_rx !== mosi_value) begin

                $display(
                    "ERROR %s RX: expected=%02h actual=%02h",
                    label,
                    mosi_value,
                    slave_rx
                );

                errors = errors + 1;

            end

        end

    endtask


    // ============================================================
    // Main tests
    // ============================================================

    initial begin

        logic [7:0] master_rx;
        logic [7:0] slave_rx;

        integer rx_count;
        integer pulse_count;
        integer i;


        // ========================================================
        // Initialization
        // ========================================================

        errors = 0;

        rst = 1'b1;

        spi_cs_n = 1'b1;
        spi_sclk = 1'b0;
        spi_mosi = 1'b0;

        tx_byte = 8'h00;
        tx_load = 1'b0;

        repeat (5) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;

        repeat (5) @(posedge clk);


        // ========================================================
        // TEST 0: Basic A5 / 3C
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 0: BASIC TRANSFER A5 / 3C");
        $display("-----------------------------------");

        errors_before = errors;

        load_tx_byte(8'h3C);
        begin_transaction();

        check_transfer(
            8'hA5,
            8'h3C,
            "basic A5/3C"
        );

        end_transaction();

        if (errors == errors_before)
            $display("TEST 0 PASSED");


        // ========================================================
        // TEST 1: Basic 5A / C3
        // ========================================================

        $display("");
        $display("----------------------------------");
        $display("TEST 1: BASIC TRANSFER 5A / C3");
        $display("-----------------------------------");

        errors_before = errors;

        load_tx_byte(8'hC3);
        begin_transaction();

        check_transfer(
            8'h5A,
            8'hC3,
            "basic 5A/C3"
        );

        end_transaction();

        if (errors == errors_before)
            $display("TEST 1 PASSED");


        // ========================================================
        // TEST 2: Extreme byte values
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 2: EXTREME BYTE VALUES");
        $display("-----------------------------------");

        errors_before = errors;


        load_tx_byte(8'hFF);
        begin_transaction();

        check_transfer(
            8'h00,
            8'hFF,
            "extreme 00/FF"
        );

        end_transaction();


        load_tx_byte(8'h00);
        begin_transaction();

        check_transfer(
            8'hFF,
            8'h00,
            "extreme FF/00"
        );

        end_transaction();


        if (errors == errors_before)
            $display("TEST 2 PASSED");


        // ========================================================
        // TEST 3: All 256 byte values
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 3: ALL 256 BYTE VALUES");
        $display("-----------------------------------");

        errors_before = errors;

        for (i = 0; i < 256; i = i + 1) begin

            load_tx_byte(~i[7:0]);
            begin_transaction();

            check_transfer(
                i[7:0],
                ~i[7:0],
                "all-values"
            );

            end_transaction();

        end

        if (errors == errors_before)
            $display("TEST 3 PASSED");


        // ========================================================
        // TEST 4: Aborted partial byte
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 4: ABORTED PARTIAL BYTE");
        $display("-----------------------------------");

        errors_before = errors;

        load_tx_byte(8'h55);
        begin_transaction();


        // Send only four bits: 1010.

        spi_mosi = 1'b1;
        #500;
        spi_sclk = 1'b1;
        #500;
        spi_sclk = 1'b0;
        #500;

        spi_mosi = 1'b0;
        #500;
        spi_sclk = 1'b1;
        #500;
        spi_sclk = 1'b0;
        #500;

        spi_mosi = 1'b1;
        #500;
        spi_sclk = 1'b1;
        #500;
        spi_sclk = 1'b0;
        #500;

        spi_mosi = 1'b0;
        #500;
        spi_sclk = 1'b1;
        #500;
        spi_sclk = 1'b0;
        #500;


        // Abort incomplete byte.
        end_transaction();


        // A completely new byte must work normally.

        load_tx_byte(8'h3C);
        begin_transaction();

        check_transfer(
            8'h5A,
            8'h3C,
            "post-abort"
        );

        end_transaction();


        if (errors == errors_before)
            $display("TEST 4 PASSED");


        // ========================================================
        // TEST 5: rx_valid exactly one FPGA clock
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 5: RX_VALID PULSE WIDTH");
        $display("-----------------------------------");

        errors_before = errors;

        load_tx_byte(8'h00);
        begin_transaction();

        transfer_byte(
            8'h96,
            master_rx,
            slave_rx,
            rx_count
        );

        if (rx_count != 1) begin

            $display(
                "ERROR rx_valid pulse count: expected=1 actual=%0d",
                rx_count
            );

            errors = errors + 1;

        end

        if (slave_rx !== 8'h96) begin

            $display(
                "ERROR rx_valid test RX: expected=96 actual=%02h",
                slave_rx
            );

            errors = errors + 1;

        end

        end_transaction();

        if (errors == errors_before)
            $display("TEST 5 PASSED");


        // ========================================================
        // TEST 6: Continuous-CS multi-byte
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 6: CONTINUOUS-CS MULTI-BYTE");
        $display("-----------------------------------");

        errors_before = errors;

        load_tx_byte(8'hB2);
        begin_transaction();


        // Byte 1

        check_transfer(
            8'hA1,
            8'hB2,
            "continuous byte 1"
        );


        // Load byte 2 without deasserting CS.

        load_tx_byte(8'hC3);

        check_transfer(
            8'hB2,
            8'hC3,
            "continuous byte 2"
        );


        // Load byte 3 without deasserting CS.

        load_tx_byte(8'hD4);

        check_transfer(
            8'hC3,
            8'hD4,
            "continuous byte 3"
        );


        end_transaction();


        if (errors == errors_before)
            $display("TEST 6 PASSED");


        // ========================================================
        // TEST 7: transaction_end exactly one FPGA clock
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 7: TRANSACTION END");
        $display("-----------------------------------");

        errors_before = errors;
        pulse_count   = 0;

        begin_transaction();

        @(negedge clk);
        spi_cs_n = 1'b1;

        repeat (10) begin

            @(negedge clk);

            if (transaction_end)
                pulse_count = pulse_count + 1;

        end


        if (pulse_count != 1) begin

            $display(
                "ERROR transaction_end pulse count: expected=1 actual=%0d",
                pulse_count
            );

            errors = errors + 1;

        end


        if (errors == errors_before)
            $display("TEST 7 PASSED");


        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("===================================");

        if (errors == 0)
            $display("ALL SPI SLAVE TESTS PASSED");
        else
            $display(
                "SPI SLAVE TESTS FAILED: %0d errors",
                errors
            );

        $display("===================================");
        $display("");

        $finish;

    end

endmodule
