module spi_slave_test (
    input  logic clk,
    input  logic rst,

    // SPI pins
    input  logic sclk,
    input  logic cs,
    input  logic mosi,
    output logic miso
);

    logic done;
    logic [7:0] received_data;
    logic [7:0] tx_data;

    // Instantiate your SPI slave
    spi_slave spi_inst (
        .clk(clk),
        .rst(rst),
        .sclk(sclk),
        .cs(cs),
        .mosi(mosi),
        .tx_data(tx_data),
        .miso(miso),
        .done(done),
        .received_data(received_data)
    );

    // Simple "application logic"
    // When we receive a byte, prepare next TX byte

    always_ff @(posedge clk) begin
        if (rst) begin
            tx_data <= 8'h00;
        end else begin
            if (done) begin
                tx_data <= received_data + 1;
            end
        end
    end

endmodule
