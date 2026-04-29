module spi_slave (
	input logic clk,
	input logic rst,
	input logic sclk,
	input logic cs,
	input logic mosi,
	input logic [7:0] tx_data,
	output logic miso,
	output logic done,
	output logic [7:0] received_data
);

	typedef enum logic [2:0] {
		IDLE,
		RECEIVING,
		DONE
	} state_t;

	state_t state, next_state;

	logic sclk_delayed;
        logic sclk_rising;
	logic sclk_falling;

        assign sclk_rising = (sclk && !sclk_delayed);
        assign sclk_falling = (!sclk && sclk_delayed);

	logic cs_delayed;
	logic cs_falling;

	assign cs_falling = (!cs && cs_delayed);

	always_ff @ (posedge clk) begin
		if (rst) begin
			state <= IDLE;
			sclk_delayed <= 0;
			cs_delayed <= 0;
		end else begin
			sclk_delayed <= sclk;
			cs_delayed <= cs;
			state <= next_state;
		end
	end

	always_comb begin
		next_state = state;
		case (state)
			IDLE: begin
				if (!rst && !cs) begin
					next_state = RECEIVING;
				end
			end

			RECEIVING: begin
				if (rst || cs) begin
					next_state = IDLE;
				end
				else if (byte_done_in) begin
					next_state = DONE;
				end
			end

			DONE: begin
				next_state = IDLE;
				
			end

		endcase
	end
	
	logic [7:0] shift_reg_in;
	logic [2:0] bit_count_in;
	logic byte_done_in;

	logic [7:0] shift_reg_out;

	assign byte_done_in = bit_count_in == 7 && sclk_rising;

	always_ff @ (posedge clk) begin
		
		if (rst) begin
		        shift_reg_in <= 0;
        		bit_count_in <= 0;
        		received_data <= 0;
			done <= 0;
			shift_reg_out <= 0;
		end else begin

			done <= 0;
			case (state)
				IDLE: begin
					bit_count_in <= 0;
					shift_reg_in <= 0;
					if (cs_falling) begin
						shift_reg_out <= tx_data;
					end
				end

				RECEIVING: begin
					if (sclk_rising) begin
						shift_reg_in <= {shift_reg_in[6:0], mosi};
						bit_count_in <= bit_count_in + 1;
					end

					if (sclk_falling) begin
						shift_reg_out <= {shift_reg_out[6:0], 0};
					end
				end

				DONE: begin
					done <= 1;
					received_data <= shift_reg_in;
				end
			endcase
		end
		
	end
	assign miso = 1'b1;			
	//assign miso = shift_reg_out[7];

endmodule
