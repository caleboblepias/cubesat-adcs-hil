module cross_product (
	input wire [31:0] wx, wy, wz,
	input wire [31:0] vx, vy, vz,
	output wire [31:0] cx, cy, cz
);

	assign cx = wy * vz - wz * vy;
	assign cy = wz * vx - wx * vz;
	assign cz = wx * vy - wy * vx;

endmodule
