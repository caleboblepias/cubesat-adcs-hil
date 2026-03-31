module eulers_rotational (
	input logic clk,
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
	input logic v_in,
	output logic [31:0] wx_dot,
	output logic [31:0] wy_dot,
	output logic [31:0] wz_dot,
	output logic v_out
);

	logic v1, v2, v3;
	logic [31:0] Iwx, Iwy, Iwz;
	logic [31:0] wx_reg, wy_reg, wz_reg;

	always_ff @(posedge clk) begin

		Iwx <= Ixx * wx;
		Iwy <= Iyy * wy;
		Iwz <= Izz * wz;

		wx_reg <= wx;
		wy_reg <= wy;
		wz_reg <= wz;

		v1 <= v_in;
	end

	logic [31:0] crossx_next, crossy_next, crossz_next;
	logic [31:0] crossx, crossy, crossz;

	cross_product cross (
		.wx(wx_reg), .wy(wy_reg), .wz(wz_reg),
		.vx(Iwx), .vy(Iwy), .vz(Iwz),
		.cx(crossx_next), .cy(crossy_next), .cz(crossz_next)
	);

	always_ff @(posedge clk) begin

		crossx <= crossx_next;
		crossy <= crossy_next;
		crossz <= crossz_next;

		v2 <= v1;
	end

	logic [31:0] diffx, diffy, diffz;
	logic [31:0] diffx_next, diffy_next, diffz_next;
	logic [31:0] torquex_reg1, torquey_reg1, torquez_reg1;
	logic [31:0] torquex_reg2, torquey_reg2, torquez_reg2;

	assign diffx_next = torquex_reg2 - crossx;
	assign diffy_next = torquey_reg2 - crossy;
	assign diffz_next = torquez_reg2 - crossz;

	always_ff @(posedge clk) begin

		torquex_reg1 <= torquex;
		torquey_reg1 <= torquey;
		torquez_reg1 <= torquez;

		torquex_reg2 <= torquex_reg1;
		torquey_reg2 <= torquey_reg1;
		torquez_reg2 <= torquez_reg1;

		diffx <= diffx_next;
		diffy <= diffy_next;
		diffz <= diffz_next;

		v3 <= v2;
	end

	always_ff @(posedge clk) begin

		wx_dot <= Ixx_inv * diffx;
		wy_dot <= Iyy_inv * diffy;
		wz_dot <= Izz_inv * diffz;

		v_out <= v3;
	end

endmodule







