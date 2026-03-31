`timescale 1ns/1ps

module tb_eulers_rotational;

    // Clock
    logic clk = 0;
    always #5 clk = ~clk; // 100 MHz

    // Inputs
    logic [31:0] Ixx = 1, Iyy = 1, Izz = 1;
    logic [31:0] Ixx_inv = 1, Iyy_inv = 1, Izz_inv = 1;

    logic [31:0] wx, wy, wz;
    logic [31:0] torquex, torquey, torquez;

    logic v_in;

    // Outputs
    logic [31:0] wx_dot, wy_dot, wz_dot;
    logic v_out;

    // DUT
    eulers_rotational dut (
        .clk(clk),
        .Ixx(Ixx), .Iyy(Iyy), .Izz(Izz),
        .Ixx_inv(Ixx_inv), .Iyy_inv(Iyy_inv), .Izz_inv(Izz_inv),
        .wx(wx), .wy(wy), .wz(wz),
        .torquex(torquex), .torquey(torquey), .torquez(torquez),
        .v_in(v_in),
        .wx_dot(wx_dot), .wy_dot(wy_dot), .wz_dot(wz_dot),
        .v_out(v_out)
    );

    initial begin
        // Initialize
        v_in = 0;
        wx = 0; wy = 0; wz = 0;
        torquex = 0; torquey = 0; torquez = 0;

        #20;

        // Send one valid sample
        wx = 10;
        wy = 5;
        wz = 2;

        torquex = 1;
        torquey = 2;
        torquez = 3;

        v_in = 1;

        #10;
        v_in = 0;

        // Let pipeline flush (4 stages → give ~10 cycles)
        repeat (10) @(posedge clk);

        $finish;
    end

endmodule

