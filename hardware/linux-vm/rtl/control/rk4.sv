module rk4 (
	input logic clk,
	input logic rst,
	input logic v_in,
	input logic [31:0] Ixx,
        input logic [31:0] Iyy,
        input logic [31:0] Izz,
        input logic [31:0] Ixx_inv,
        input logic [31:0] Iyy_inv,
        input logic [31:0] Izz_inv,
        input logic [31:0] wx,
        input logic [31:0] wy,
        input logic [31:0] wz,
        input logic [31:0] torquex,
        input logic [31:0] torquey,
        input logic [31:0] torquez,
	input logic [31:0] dt,
	output logic [31:0] wx_out,
	output logic [31:0] wy_out,
	output logic [31:0] wz_out
);

	typedef enum logic [2:0] {
		IDLE,
		K1,
		K2,
		K3,
		K4,
		ESTIMATE,
		FINAL,
		DONE
	} state_t;

	state_t state, next_state;

	always_ff @(posedge clk) begin
		if (rst)
			state <= IDLE;
		else 
			state <= next_state;
	end

	logic v_in_sig, v_out_sig;
	logic [31:0] wx_reg, wy_reg, wz_reg;
	logic [31:0] wx_dot_reg, wy_dot_reg, wz_dot_reg;

	eulers_rotational eulers_instance (
		.clk(clk),
		.Ixx(Ixx), .Iyy(Iyy), .Izz(Izz),
		.Ixx_inv(Ixx_inv), .Iyy_inv(Iyy_inv), .Izz_inv(Izz_inv),
		.wx(wx_reg), .wy(wy_reg), .wz(wz_reg),
		.torquex(torquex), .torquey(torquey), .torquez(torquez),
		.v_in(v_in_sig),
		.wx_dot(wx_dot_reg), .wy_dot(wy_dot_reg), .wz_dot(wz_dot_reg),
		.v_out(v_out_sig)
	);

	always_comb begin
		next_state = state;
		case(state)
			IDLE: begin
				if (v_in)
					next_state = K1;
			end

			K1: begin
				if (v_out_sig)
					next_state = K2;

			end

			K2: begin
				if (v_out_sig)
					next_state = K3;

			end

			K3: begin
				if (v_out_sig)
					next_state = K4;

			end
	
			K4: begin
				if (v_out_sig)
					next_state = ESTIMATE;

			end

			ESTIMATE: begin
				
				next_state = FINAL;

			end

			FINAL: begin
				next_state = DONE;
			end
			DONE: begin

			end



		endcase
	end

	state_t state_prev;

	always_ff @(posedge clk) begin
		if (rst) begin
			state_prev <= IDLE;
			v_in_sig <= 0;
		end else begin
			state_prev <= state;
			v_in_sig <= (state != state_prev) && 
				(state == K1 || state == K2 || state == K3 || state == K4);
		end
	end

	logic [31:0] wx_dot_k1, wy_dot_k1, wz_dot_k1;
	logic [31:0] wx_dot_k2, wy_dot_k2, wz_dot_k2;
	logic [31:0] wx_dot_k3, wy_dot_k3, wz_dot_k3;
	logic [31:0] wx_dot_k4, wy_dot_k4, wz_dot_k4;
	logic [31:0] sumx, sumy, sumz;
	logic [63:0] multx, multy, multz;

	localparam ONE_SIXTH = 32'd10923;

	assign multx = sumx * ONE_SIXTH;
	assign multy = sumy * ONE_SIXTH;
        assign multz = sumz * ONE_SIXTH;

	always_ff @(posedge clk) begin
		if (rst) begin
			wx_reg <= 0;
			wy_reg <= 0;
			wz_reg <= 0;
			wx_dot_k1 <= 0;
                        wy_dot_k1 <= 0;
                        wz_dot_k1 <= 0;
			wx_dot_k2 <= 0;         
                        wy_dot_k2 <= 0;         
                        wz_dot_k2 <= 0;
			wx_dot_k3 <= 0;         
                        wy_dot_k3 <= 0;         
                        wz_dot_k3 <= 0;
			wx_dot_k4 <= 0;         
                        wy_dot_k4 <= 0;         
                        wz_dot_k4 <= 0;
			wx_out <= 0;
			wy_out <= 0;
			wz_out <= 0;
		end else begin
			
			case(state)
				K1: begin

					wx_reg <= wx;
					wy_reg <= wy;
					wz_reg <= wz;
					
					if (v_out_sig) begin
						wx_dot_k1 <= wx_dot_reg;
						wy_dot_k1 <= wy_dot_reg;
						wz_dot_k1 <= wz_dot_reg;
					end
				end

				K2: begin
					wx_reg <= wx + (dt >> 1) * wx_dot_k1;
					wy_reg <= wy + (dt >> 1) * wy_dot_k1;
					wz_reg <= wz + (dt >> 1) * wz_dot_k1;
					
					if (v_out_sig) begin
                                                wx_dot_k2 <= wx_dot_reg;
                                                wy_dot_k2 <= wy_dot_reg;
                                                wz_dot_k2 <= wz_dot_reg;
					end
				end

				K3: begin
					wx_reg <= wx + (dt >> 1) * wx_dot_k2;
                                        wy_reg <= wy + (dt >> 1) * wy_dot_k2;
                                        wz_reg <= wz + (dt >> 1) * wz_dot_k2;

					if (v_out_sig) begin
                                                wx_dot_k3 <= wx_dot_reg;
                                                wy_dot_k3 <= wy_dot_reg;
                                                wz_dot_k3 <= wz_dot_reg;
					end

				end

				K4: begin
					wx_reg <= wx + dt * wx_dot_k3;
                                        wy_reg <= wy + dt * wy_dot_k3;
                                        wz_reg <= wz + dt * wz_dot_k3;

					if (v_out_sig) begin
                                                wx_dot_k4 <= wx_dot_reg;
                                                wy_dot_k4 <= wy_dot_reg;
                                                wz_dot_k4 <= wz_dot_reg;
					end

				end

				ESTIMATE: begin
					/*
					wx_out <= wx + (dt / 6) * (wx_dot_k1 + 2 * wx_dot_k2 + 2 * wx_dot_k3 + wx_dot_k4);
					wy_out <= wy + (dt / 6) * (wy_dot_k1 + 2 * wy_dot_k2 + 2 * wy_dot_k3 + wy_dot_k4);
					wz_out <= wz + (dt / 6) * (wz_dot_k1 + 2 * wz_dot_k2 + 2 * wz_dot_k3 + wz_dot_k4);
					*/
				        /*
				        wx_out <= wx + wx_dot_k1;
				        wy_out <= wy + wy_dot_k1;
				        wz_out <= wz + wz_dot_k1;
					*/
				       	sumx <= wx_dot_k1 + (wx_dot_k2 << 1) + (wx_dot_k3 << 1) + wx_dot_k4;
        				sumy <= wy_dot_k1 + (wy_dot_k2 << 1) + (wy_dot_k3 << 1) + wy_dot_k4;
        				sumz <= wz_dot_k1 + (wz_dot_k2 << 1) + (wz_dot_k3 << 1) + wz_dot_k4;
				end

				FINAL: begin

                                        wx_out <= wx + (multx >> 16);
                                        wy_out <= wy + (multy >> 16);
                                        wz_out <= wz + (multz >> 16);
				end


			endcase
		end


				
	end
endmodule

	






























