module spi_slave_sclk (
	input  logic       sclk,
	input  logic       cs,
    	input  logic       mosi,
    	input  logic [7:0] tx_data,
   	output logic       miso,
    	output logic       done,
    	output logic [7:0] received_data
);

    	logic [7:0] shift_reg_in;
    	logic [7:0] shift_reg_out;
    	logic [2:0] bit_count_in;
	logic [7:0] next_shift;

    	always_ff @(posedge sclk or posedge cs) begin
        	if (cs) begin
            		bit_count_in  <= 3'd0;
            		shift_reg_in  <= 8'd0;
            		received_data <= 8'd0;
            		done          <= 1'b0;
        	end else begin
			next_shift = {shift_reg_in[6:0], mosi};
          		shift_reg_in <= next_shift;

            		if (bit_count_in == 3'd7) begin
                		bit_count_in  <= 3'd0;
                		//received_data <= {shift_reg_in[6:0], mosi};
				received_data <= next_shift;
                		done          <= 1'b1;
            		end else begin
                		bit_count_in <= bit_count_in + 3'd1;
                		done         <= 1'b0;
            		end
        	end
    	end
	
	logic [2:0] bit_count_out;

    	always_ff @(negedge sclk or posedge cs) begin
        	if (cs) begin
			//bit_count_out <= 3'd7;
            		shift_reg_out <= tx_data;
		end else if (done) begin
			/*
			if (bit_count_out == 3'd7) begin
				bit_count_out <= 3'd0;
				shift_reg_out <= tx_data;


        		end else begin
				bit_count_out <= bit_count_out + 3'd1;
            			shift_reg_out <= {shift_reg_out[6:0], 1'b0};
			end
			*/
		       shift_reg_out <= tx_data;
		end else begin
		       shift_reg_out <= {shift_reg_out[6:0], 1'b0};
        	end
    	end

    	assign miso = shift_reg_out[7];

	
endmodule
