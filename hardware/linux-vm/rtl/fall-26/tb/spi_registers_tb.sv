`timescale 1ns / 1ps

module spi_registers_tb;

    // ============================================================
    // DUT signals
    // ============================================================

    logic clk;
    logic rst;

    logic [7:0] rx_byte;
    logic       rx_valid;

    logic [7:0] tx_byte;
    logic       tx_load;

    logic transaction_end;

    logic busy;
    logic ready;

    logic start_request;

    logic state_update;

    logic signed [31:0] wx_next;
    logic signed [31:0] wy_next;
    logic signed [31:0] wz_next;

    logic signed [31:0] torque_x;
    logic signed [31:0] torque_y;
    logic signed [31:0] torque_z;

    logic signed [31:0] wx;
    logic signed [31:0] wy;
    logic signed [31:0] wz;

    logic signed [31:0] dt;

    logic signed [31:0] ixx;
    logic signed [31:0] iyy;
    logic signed [31:0] izz;

    logic signed [31:0] ixx_inv;
    logic signed [31:0] iyy_inv;
    logic signed [31:0] izz_inv;

    integer errors;


    // ============================================================
    // Protocol constants
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
    // DUT
    // ============================================================

    spi_registers dut (
        .clk             (clk),
        .rst             (rst),

        .rx_byte         (rx_byte),
        .rx_valid        (rx_valid),

        .tx_byte         (tx_byte),
        .tx_load         (tx_load),

        .transaction_end (transaction_end),

        .busy            (busy),
        .ready           (ready),

        .start_request   (start_request),

        .state_update    (state_update),

        .wx_next         (wx_next),
        .wy_next         (wy_next),
        .wz_next         (wz_next),

        .torque_x        (torque_x),
        .torque_y        (torque_y),
        .torque_z        (torque_z),

        .wx              (wx),
        .wy              (wy),
        .wz              (wz),

        .dt              (dt),

        .ixx             (ixx),
        .iyy             (iyy),
        .izz             (izz),

        .ixx_inv         (ixx_inv),
        .iyy_inv         (iyy_inv),
        .izz_inv         (izz_inv)
    );


    // ============================================================
    // 100 MHz clock
    // ============================================================

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end


    // ============================================================
    // Send one protocol byte
    // ============================================================

    task automatic send_byte (
        input logic [7:0] value
    );

        begin

            @(negedge clk);

            rx_byte  = value;
            rx_valid = 1'b1;

            @(negedge clk);

            rx_valid = 1'b0;

        end

    endtask


    // ============================================================
    // End current transaction
    // ============================================================

    task automatic end_transaction;

        begin

            @(negedge clk);
            transaction_end = 1'b1;

            @(negedge clk);
            transaction_end= 1'b0;

        end

    endtask


    // ============================================================
    // Write one 32-bit register
    // ============================================================

    task automatic write_register (
        input logic [7:0]  addr,
        input logic [31:0] value
    );

        begin

            send_byte(CMD_WRITE);
            send_byte(addr);

            send_byte(value[31:24]);
            send_byte(value[23:16]);
            send_byte(value[15:8]);
            send_byte(value[7:0]);

        end

    endtask


    // ============================================================
    // Read one 32-bit register
    // ============================================================

    task automatic read_register (
        input  logic [7:0]  addr,
        output logic [31:0] value
    );

        begin

            value = 32'h00000000;

            send_byte(CMD_READ);

            // Address snapshots the register and loads first byte.
            send_byte(addr);

            if (tx_load !== 1'b1) begin
                $display(
                    "ERROR: tx_load missing for read byte [31:24]"
                );
                errors = errors + 1;
            end

            value[31:24] = tx_byte;


            send_byte(8'h00);

            if (tx_load !== 1'b1) begin
                $display(
                    "ERROR: tx_load missing for read byte [23:16]"
                );
                errors = errors + 1;
            end

            value[23:16] = tx_byte;


            send_byte(8'h00);

            if (tx_load !== 1'b1) begin
                $display(
                    "ERROR: tx_load missing for read byte [15:8]"
                );
                errors = errors + 1;
            end

            value[15:8] = tx_byte;


            send_byte(8'h00);

            if (tx_load !== 1'b1) begin
                $display(
                    "ERROR: tx_load missing for read byte [7:0]"
                );
                errors = errors + 1;
            end

            value[7:0] = tx_byte;


            // Final dummy returns protocol FSM to WAIT_CMD.
            send_byte(8'h00);

        end

    endtask


    // ============================================================
    // Value checker
    // ============================================================

    task automatic check_value (
        input logic [31:0] actual,
        input logic [31:0] expected,
        input string       name
    );

        begin

            if (actual !== expected) begin

                $display(
                    "ERROR %s: expected=0x%08h actual=0x%08h",
                    name,
                    expected,
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

        logic [31:0] read_value;
        integer errors_before;
        integer pulse_count;


        // ========================================================
        // Initialization
        // ========================================================

        errors = 0;

        rst      = 1'b1;
        rx_byte  = 8'h00;
        rx_valid = 1'b0;

        transaction_end = 1'b0;

        busy  = 1'b0;
        ready = 1'b0;

        state_update = 1'b0;

        wx_next = '0;
        wy_next = '0;
        wz_next = '0;


        repeat (5) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;


        // ========================================================
        // TEST 1: Register writes
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 1: REGISTER WRITES");
        $display("-----------------------------------");

        errors_before = errors;

        write_register(ADDR_TORQUE_X, 32'h12345678);
        write_register(ADDR_TORQUE_Y, 32'h87654321);
        write_register(ADDR_TORQUE_Z, 32'hDEADBEEF);

        write_register(ADDR_WX, 32'h11112222);
        write_register(ADDR_WY, 32'h33334444);
        write_register(ADDR_WZ, 32'h55556666);

        write_register(ADDR_DT, 32'h0000028F);

        write_register(ADDR_IXX, 32'h0000051F);
        write_register(ADDR_IYY, 32'h00000520);
        write_register(ADDR_IZZ, 32'h0000028F);

        write_register(ADDR_IXX_INV, 32'h00320000);
        write_register(ADDR_IYY_INV, 32'h00330000);
        write_register(ADDR_IZZ_INV, 32'h00640000);


        check_value(torque_x, 32'h12345678, "TORQUE_X");
        check_value(torque_y, 32'h87654321, "TORQUE_Y");
        check_value(torque_z, 32'hDEADBEEF, "TORQUE_Z");

        check_value(wx, 32'h11112222, "WX");
        check_value(wy, 32'h33334444, "WY");
        check_value(wz, 32'h55556666, "WZ");

        check_value(dt, 32'h0000028F, "DT");

        check_value(ixx, 32'h0000051F, "IXX");
        check_value(iyy, 32'h00000520, "IYY");
        check_value(izz, 32'h0000028F, "IZZ");

        check_value(ixx_inv, 32'h00320000, "IXX_INV");
        check_value(iyy_inv, 32'h00330000, "IYY_INV");
        check_value(izz_inv, 32'h00640000, "IZZ_INV");


        if (errors == errors_before)
            $display("TEST 1 PASSED");


        // ========================================================
        // TEST 2: Readback / byte ordering
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 2: READBACK / BYTE ORDER");
        $display("-----------------------------------");

        errors_before = errors;


        read_register(ADDR_TORQUE_X, read_value);
        check_value(
            read_value,
            32'h12345678,
            "TORQUE_X readback"
        );


        read_register(ADDR_TORQUE_Y, read_value);
        check_value(
            read_value,
            32'h87654321,
            "TORQUE_Y readback"
        );


        read_register(ADDR_WX, read_value);
        check_value(
            read_value,
            32'h11112222,
            "WX readback"
        );


        read_register(ADDR_DT, read_value);
        check_value(
            read_value,
            32'h0000028F,
            "DT readback"
        );


        if (errors == errors_before)
            $display("TEST 2 PASSED");


        // ========================================================
        // TEST 3: STATUS register
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 3: STATUS REGISTER");
        $display("-----------------------------------");

        errors_before = errors;


        busy  = 1'b0;
        ready = 1'b0;

        read_register(ADDR_STATUS, read_value);

        check_value(
            read_value,
            32'h00000000,
            "STATUS idle"
        );


        busy  = 1'b1;
        ready = 1'b0;

        read_register(ADDR_STATUS, read_value);

        check_value(
            read_value,
            32'h00000001,
            "STATUS busy"
        );


        busy  = 1'b0;
        ready = 1'b1;

        read_register(ADDR_STATUS, read_value);

        check_value(
            read_value,
            32'h00000002,
            "STATUS ready"
        );


        busy  = 1'b1;
        ready = 1'b1;

        read_register(ADDR_STATUS, read_value);

        check_value(
            read_value,
            32'h00000003,
            "STATUS busy+ready"
        );


        busy  = 1'b0;
        ready = 1'b0;


        if (errors == errors_before)
            $display("TEST 3 PASSED");


        // ========================================================
        // TEST 4: CONTROL.START pulse
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 4: START REQUEST PULSE");
        $display("-----------------------------------");

        errors_before = errors;
        pulse_count   = 0;


        send_byte(CMD_WRITE);
        send_byte(ADDR_CONTROL);

        send_byte(8'h00);
        send_byte(8'h00);
        send_byte(8'h00);

        send_byte(8'h01);


        if (start_request !== 1'b1) begin

            $display(
                "ERROR: start_request not asserted"
            );

            errors = errors + 1;

        end else begin

            pulse_count = pulse_count + 1;

        end


        @(posedge clk);
        @(negedge clk);

        if (start_request === 1'b1)
            pulse_count = pulse_count + 1;


        if (pulse_count != 1) begin

            $display(
                "ERROR: start_request pulse count = %0d",
                pulse_count
            );

            errors = errors + 1;

        end


        write_register(
            ADDR_CONTROL,
            32'h00000000
        );


        if (start_request !== 1'b0) begin

            $display(
                "ERROR: CONTROL=0 asserted start_request"
            );

            errors = errors + 1;

        end


        if (errors == errors_before)
            $display("TEST 4 PASSED");


        // ========================================================
        // TEST 5: STATUS read-only
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 5: STATUS READ-ONLY");
        $display("-----------------------------------");

        errors_before = errors;

        busy  = 1'b0;
        ready = 1'b1;


        write_register(
            ADDR_STATUS,
            32'hFFFFFFFF
        );


        read_register(
            ADDR_STATUS,
            read_value
        );


        check_value(
            read_value,
            32'h00000002,
            "STATUS after write"
        );


        busy  = 1'b0;
        ready = 1'b0;


        if (errors == errors_before)
            $display("TEST 5 PASSED");


        // ========================================================
        // TEST 6: Invalid address
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 6: INVALID ADDRESS");
        $display("-----------------------------------");

        errors_before = errors;


        write_register(
            8'hFE,
            32'hDEADBEEF
        );


        check_value(
            torque_x,
            32'h12345678,
            "TORQUE_X after invalid write"
        );


        check_value(
            wx,
            32'h11112222,
            "WX after invalid write"
        );


        read_register(
            8'hFE,
            read_value
        );


        check_value(
            read_value,
            32'h00000000,
            "invalid address read"
        );


        if (errors == errors_before)
            $display("TEST 6 PASSED");


        // ========================================================
        // TEST 7: Invalid command recovery
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 7: INVALID COMMAND RECOVERY");
        $display("-----------------------------------");

        errors_before = errors;


        send_byte(8'hAA);


        write_register(
            ADDR_TORQUE_X,
            32'hCAFEBABE
        );


        check_value(
            torque_x,
            32'hCAFEBABE,
            "TORQUE_X after invalid command"
        );


        if (errors == errors_before)
            $display("TEST 7 PASSED");


        // ========================================================
        // TEST 8: Read snapshot coherence
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 8: READ SNAPSHOT");
        $display("-----------------------------------");

        errors_before = errors;

        busy  = 1'b1;
        ready = 1'b0;

        read_value = 32'h00000000;


        send_byte(CMD_READ);

        send_byte(ADDR_STATUS);

        if (tx_load !== 1'b1) begin
            $display(
                "ERROR: snapshot first tx_load missing"
            );
            errors = errors + 1;
        end

        read_value[31:24] = tx_byte;


        busy  = 1'b0;
        ready = 1'b1;


        send_byte(8'h00);

        if (tx_load !== 1'b1) begin
            $display(
                "ERROR: snapshot tx_load [23:16] missing"
            );
            errors = errors + 1;
        end

        read_value[23:16] = tx_byte;


        send_byte(8'h00);

        if (tx_load !== 1'b1) begin
            $display(
                "ERROR: snapshot tx_load [15:8] missing"
            );
            errors = errors + 1;
        end

        read_value[15:8] = tx_byte;


        send_byte(8'h00);

        if (tx_load !== 1'b1) begin
            $display(
                "ERROR: snapshot tx_load [7:0] missing"
            );
            errors = errors + 1;
        end

        read_value[7:0] = tx_byte;


        send_byte(8'h00);


        check_value(
            read_value,
            32'h00000001,
            "STATUS snapshot"
        );


        read_register(
            ADDR_STATUS,
            read_value
        );


        check_value(
            read_value,
            32'h00000002,
            "new STATUS snapshot"
        );


        busy  = 1'b0;
        ready = 1'b0;


        if (errors == errors_before)
            $display("TEST 8 PASSED");


        // ========================================================
        // TEST 9: Hardware state update
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 9: HARDWARE STATE UPDATE");
        $display("-----------------------------------");

        errors_before = errors;


        write_register(
            ADDR_WX,
            32'h00010000
        );

        write_register(
            ADDR_WY,
            32'hFFFE0000
        );

        write_register(
            ADDR_WZ,
            32'h00030000
        );


        @(negedge clk);

        wx_next = 32'sh00011000;
        wy_next = 32'shFFFE2000;
        wz_next = 32'sh00033000;

        state_update = 1'b1;


        @(posedge clk);

        @(negedge clk);
        state_update = 1'b0;


        check_value(
            wx,
            32'h00011000,
            "WX hardware update"
        );

        check_value(
            wy,
            32'hFFFE2000,
            "WY hardware update"
        );

        check_value(
            wz,
            32'h00033000,
            "WZ hardware update"
        );


        read_register(ADDR_WX, read_value);

        check_value(
            read_value,
            32'h00011000,
            "WX readback after hardware update"
        );


        read_register(ADDR_WY, read_value);

        check_value(
            read_value,
            32'hFFFE2000,
            "WY readback after hardware update"
        );


        read_register(ADDR_WZ, read_value);

        check_value(
            read_value,
            32'h00033000,
            "WZ readback after hardware update"
        );


        if (errors == errors_before)
            $display("TEST 9 PASSED");


        // ========================================================
        // TEST 10: Software state reinitialization
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 10: SOFTWARE STATE REINIT");
        $display("-----------------------------------");

        errors_before = errors;


        write_register(
            ADDR_WX,
            32'h000A0000
        );

        write_register(
            ADDR_WY,
            32'hFFF50000
        );

        write_register(
            ADDR_WZ,
            32'h000C0000
        );


        check_value(
            wx,
            32'h000A0000,
            "WX software reinit"
        );

        check_value(
            wy,
            32'hFFF50000,
            "WY software reinit"
        );

        check_value(
            wz,
            32'h000C0000,
            "WZ software reinit"
        );


        if (errors == errors_before)
            $display("TEST 10 PASSED");


        // ========================================================
        // TEST 11: Hardware update priority
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 11: STATE UPDATE PRIORITY");
        $display("-----------------------------------");

        errors_before = errors;


        send_byte(CMD_WRITE);
        send_byte(ADDR_WX);

        send_byte(8'hDE);
        send_byte(8'hAD);
        send_byte(8'hBE);


        @(negedge clk);

        rx_byte  = 8'hEF;
        rx_valid = 1'b1;

        wx_next = 32'sh01234567;
        wy_next = 32'sh89ABCDEF;
        wz_next = 32'sh13572468;

        state_update = 1'b1;


        @(posedge clk);

        @(negedge clk);

        rx_valid      = 1'b0;
        state_update  = 1'b0;


        check_value(
            wx,
            32'h01234567,
            "WX hardware priority"
        );

        check_value(
            wy,
            32'h89ABCDEF,
            "WY atomic priority update"
        );

        check_value(
            wz,
            32'h13572468,
            "WZ atomic priority update"
        );


        if (errors == errors_before)
            $display("TEST 11 PASSED");


        // ========================================================
        // TEST 12: Aborted transaction recovery
        //
        // An incomplete write must not commit. transaction_end
        // must return the protocol FSM to WAIT_CMD so that the
        // next transaction starts cleanly.
        // ========================================================

        $display("");
        $display("-----------------------------------");
        $display("TEST 12: ABORTED TRANSACTION");
        $display("-----------------------------------");

        errors_before = errors;


        // Establish known register value.
        write_register(
            ADDR_TORQUE_X,
            32'h11223344
        );


        // Begin a new write to the same register.
        send_byte(CMD_WRITE);
        send_byte(ADDR_TORQUE_X);

        // Only half the payload arrives.
        send_byte(8'hDE);
        send_byte(8'hAD);


        // CS rises before the command is complete.
        end_transaction();


        // Partial write must not commit.
        check_value(
            torque_x,
            32'h11223344,
            "TORQUE_X after aborted write"
        );


        // Next command must start cleanly.
        write_register(
            ADDR_TORQUE_X,
            32'hCAFEBABE
        );


        check_value(
            torque_x,
            32'hCAFEBABE,
            "TORQUE_X after abort recovery"
        );


        // And the read path must still be aligned.
        read_register(
            ADDR_TORQUE_X,
            read_value
        );


        check_value(
            read_value,
            32'hCAFEBABE,
            "TORQUE_X read after abort recovery"
        );


        if (errors == errors_before)
            $display("TEST 12 PASSED");


        // ========================================================
        // Final result
        // ========================================================

        $display("");
        $display("===================================");

        if (errors == 0) begin

            $display(
                "ALL SPI REGISTER TESTS PASSED"
            );

        end else begin

            $display(
                "SPI REGISTER TESTS FAILED: %0d errors",
                errors
            );

        end

        $display("===================================");
        $display("");

        $finish;

    end

endmodule 
