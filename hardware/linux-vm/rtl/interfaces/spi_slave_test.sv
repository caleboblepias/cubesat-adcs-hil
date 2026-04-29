module spi_slave_test (
    	input  logic clk,

	// SPI pins
	input  logic sclk,
	input  logic cs,
	input  logic mosi,
	output logic miso
);

	logic done;
	logic [7:0] received_data;
	logic [7:0] tx_data;


    	spi_slave spi_inst (
        	.clk(clk),
        	.rst(1'b0),
        	.sclk(sclk),
        	.cs(cs),	
		.mosi(mosi),
		.tx_data(tx_data),
		.miso(miso),
		.done(done),
		.received_data(received_data)
	);
	
	assign tx_data = 8'hA5;
	// When we receive a byte, prepare next TX byte
    /*
    logic rst_reg = 1;
    assign rst = rst_reg;
    always_ff @(posedge clk) begin
	    rst_reg <= 0;
    end
    always_ff @(posedge clk) begin
        if (rst) begin
            tx_data <= 8'h00;
        end else begin
            if (done) begin
                tx_data <= received_data + 1;
            end
        end
    end
    */
    

endmodule
