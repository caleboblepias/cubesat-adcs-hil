`timescale 1ns/1ps

module spi_protocol_tb;

    // ================= SPI SIGNALS =================
    logic sclk;
    logic cs;
    logic mosi;
    logic miso;

    // ================= PHY SIGNALS =================
    logic [7:0] rx_data;
    logic       rx_valid;
    logic [7:0] tx_data;

    // ================= PROTOCOL SIGNALS =================
    logic        we;
    logic [7:0]  waddr;
    logic [7:0]  wdata;

    logic [7:0]  raddr;
    logic [7:0]  rdata;

    // ================= REGISTER FILE =================
    logic [7:0] regs [0:255];

    // ================= READ CAPTURE =================
    logic [7:0] d0, d1, d2;

    // ================= INIT =================
    initial begin
        integer i;
        for (i = 0; i < 256; i = i + 1) begin
            regs[i] = i;   // debug pattern
        end
    end

    // ================= WRITE LOGIC =================
    always_ff @(posedge sclk) begin
        if (we)
            regs[waddr] <= wdata;
    end

    assign rdata = regs[raddr];

    // ================= DUT =================
    spi_slave_sclk spi_phy (
        .sclk(sclk),
        .cs(cs),
        .mosi(mosi),
        .tx_data(tx_data),
        .miso(miso),
        .done(rx_valid),
        .received_data(rx_data)
    );

    spi_protocol dut (
        .sclk(sclk),
        .cs(cs),
        .rx_data(rx_data),
        .rx_valid(rx_valid),

        .we(we),
        .waddr(waddr),
        .wdata(wdata),

        .raddr(raddr),
        .rdata(rdata),

        .tx_data(tx_data)
    );

    // ================= CLOCK TASK =================
    task spi_clock;
        begin
            #5 sclk = 0;
            #5 sclk = 1;
        end
    endtask

    // ================= SEND BYTE =================
    task spi_send_byte(input [7:0] data);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                mosi = data[i];
                spi_clock();
            end
        end
    endtask

    // ================= READ BYTE =================
    task spi_read_byte(output [7:0] data);
        integer i;
        begin
            data = 8'd0;
            for (i = 7; i >= 0; i = i - 1) begin
                mosi = 0;
                #5 sclk = 0;
                #5 sclk = 1;
                data[i] = miso;
            end
        end
    endtask

    // ================= TEST =================
    initial begin
        // init signals
        sclk = 0;
        cs   = 1;
        mosi = 0;

        #20;

        // ================= WRITE =================
        $display("---- WRITE TEST ----");

        cs = 0;
        spi_send_byte(8'h01); // CMD write
        spi_send_byte(8'h05); // ADDR
        spi_send_byte(8'hA7); // DATA
        cs = 1;

        #40;

        // ================= READ =================
        $display("---- READ TEST ----");

        cs = 0;
        spi_send_byte(8'h02); // CMD read
        spi_send_byte(8'h05); // ADDR

        spi_read_byte(d0);
        spi_read_byte(d1);
        spi_read_byte(d2);

        cs = 1;

        $display("RX bytes: %h %h %h", d0, d1, d2);

        #50;
        $finish;
    end

endmodule
