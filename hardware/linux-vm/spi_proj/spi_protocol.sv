module spi_protocol (

	input logic sclk,
	input logic cs,
	input logic [7:0] rx_data,
	input logic rx_valid,
	
	// WRITE interface
	output logic we,
	output logic [7:0] waddr,
	output logic [7:0] wdata,

	// READ interface
	output logic [7:0] raddr,
	input logic [7:0] rdata,

	// back to PHY
	output logic [7:0] tx_data

);

	typedef enum logic [1:0] {
		WAIT_CMD,
		WAIT_ADDR,
		WAIT_DATA
	} state_t;

	state_t state, next_state;

	logic [7:0] cmd;
	logic [7:0] addr;
	
	always_ff @(posedge sclk) begin
		if (cs) begin
			state <= WAIT_CMD;
		end else begin
			state <= next_state;
		end
	end

	always_comb begin
		next_state = state;
		case (state)
			WAIT_CMD: begin
				if (rx_valid) begin
					next_state = WAIT_ADDR;
				end
			end

			WAIT_ADDR: begin
				if (rx_valid) begin
					if (cmd == 8'h01) begin
						next_state = WAIT_DATA;
					end else if (cmd == 8'h02) begin
						next_state = WAIT_CMD;
					end
				end
			end

			WAIT_DATA: begin
				if (rx_valid) begin
					next_state = WAIT_CMD;
				end
			end
		endcase
	end

	logic read_pending;

	always_ff @(posedge sclk) begin
		if (cs) begin
			cmd <= 0;
			addr <= 0;

			we <= 0;
			waddr <= 0;
			wdata <= 0;

			raddr <= 0;

			tx_data <= 0;

			read_pending <= 0;
		end else begin
			we <= 0;

			// pending read handling
			if (read_pending && rx_valid) begin
				tx_data <= rdata;
				read_pending <= 0;
			end
			case (state)
				WAIT_CMD: begin
					if (rx_valid) begin
						cmd <= rx_data;
					end
				end

				WAIT_ADDR: begin
					if (rx_valid) begin
						addr <= rx_data;
						raddr <= rx_data;

						if (cmd == 8'h02) begin
							read_pending <= 1;
						end
					end
				end

				WAIT_DATA: begin
					if (rx_valid) begin
						we <= 1;
						waddr <= addr;
						wdata <= rx_data;
					end
				end
				
			endcase

		end
	end
endmodule



