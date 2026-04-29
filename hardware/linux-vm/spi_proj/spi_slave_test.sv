module spi_slave_test (

	// SPI pins
	input  logic sclk,
	input  logic cs,
	input  logic mosi,
	output logic miso,

	output logic done,
	output logic [2:0] debug_received
);


	logic [7:0] received_data;
	logic [7:0] tx_data;


    	spi_slave_sclk spi_inst (
        	.sclk(sclk),
        	.cs(cs),	
		.mosi(mosi),
		.tx_data(tx_data),
		.miso(miso),
		.done(done),
		.received_data(received_data)
	);
	
	always_ff @(posedge sclk or posedge cs) begin
                if (cs)
                        tx_data <= 8'h01;
                else if (done)
                        tx_data <= tx_data + 1;
        end
	assign debug_received = received_data[2:0];
	
    

endmodule
