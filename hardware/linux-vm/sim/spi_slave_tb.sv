`timescale 1ns/1ps

module spi_slave_tb;

    logic clk;
    logic rst;
    logic sclk;
    logic cs;
    logic mosi;
    logic miso;
    logic done;
    logic [7:0] received_data;

    // Instantiate DUT
    spi_slave dut (
        .clk(clk),
        .rst(rst),
        .sclk(sclk),
        .cs(cs),
        .mosi(mosi),
        .miso(miso),
        .done(done),
        .received_data(received_data)
    );

    // -------------------------
    // System clock (100 MHz)
    // -------------------------
    initial clk = 0;
    always #5 clk = ~clk;  // 10ns period

    // -------------------------
    // SPI clock (slower)
    // -------------------------
    task spi_clock_tick();
        begin
            #20 sclk = 1;
            #20 sclk = 0;
        end
    endtask

    // -------------------------
    // Send one byte (MSB first)
    // -------------------------
    task spi_send_byte(input [7:0] data);
        integer i;
        begin
            for (i = 7; i >= 0; i--) begin
                mosi = data[i];
                spi_clock_tick();
            end
        end
    endtask

    // -------------------------
    // Test sequence
    // -------------------------
    initial begin
        // Initialize
        clk = 0;
        sclk = 0;
        cs = 1;
        mosi = 0;
        rst = 1;

        #50;
        rst = 0;

        #50;

        // Start transaction
        cs = 0;

        // Send byte
        spi_send_byte(8'hA5);

        // End transaction
        cs = 1;

        #100;

        // Check result
        if (received_data == 8'hA5)
            $display("PASS: Received %h", received_data);
        else
            $display("FAIL: Got %h, expected A5", received_data);

        #100;
        $finish;
    end

endmodule
