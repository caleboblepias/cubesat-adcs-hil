module spi_protocol_test (

        // SPI pins
        input  logic sclk,
        input  logic cs,
        input  logic mosi,
        output logic miso,

        // debug pins (match your XDC)
        output logic done,
        output logic [2:0] debug_received

);

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

        // ================= FAKE REGISTER FILE =================
        logic [7:0] regs [0:255];

        // simple memory behavior
        always_ff @(posedge sclk) begin
                if (we)
                        regs[waddr] <= wdata;
        end

        assign rdata = regs[raddr];

        // ================= PHY =================
        spi_slave_sclk spi_phy (
                .sclk(sclk),
                .cs(cs),
                .mosi(mosi),
                .tx_data(tx_data),
                .miso(miso),
                .done(rx_valid),
                .received_data(rx_data)
        );

        // ================= PROTOCOL =================
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

        // ================= DEBUG =================
        assign done = rx_valid;

        // show last received byte LSBs
        assign debug_received = rx_data[2:0];

endmodule
