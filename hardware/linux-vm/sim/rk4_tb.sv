`timescale 1ns/1ps

module rk4_tb;

    // =============================
    // DUT signals
    // =============================
    logic clk;
    logic rst;
    logic v_in;

    logic [31:0] Ixx, Iyy, Izz;
    logic [31:0] Ixx_inv, Iyy_inv, Izz_inv;

    logic [31:0] wx, wy, wz;
    logic [31:0] torquex, torquey, torquez;
    logic [31:0] dt;

    logic [31:0] wx_out, wy_out, wz_out;

    // =============================
    // Instantiate DUT
    // =============================
    rk4 dut (
        .clk(clk),
        .rst(rst),
        .v_in(v_in),

        .Ixx(Ixx), .Iyy(Iyy), .Izz(Izz),
        .Ixx_inv(Ixx_inv), .Iyy_inv(Iyy_inv), .Izz_inv(Izz_inv),

        .wx(wx), .wy(wy), .wz(wz),
        .torquex(torquex), .torquey(torquey), .torquez(torquez),

        .dt(dt),

        .wx_out(wx_out),
        .wy_out(wy_out),
        .wz_out(wz_out)
    );

    // =============================
    // Clock generation (10ns period)
    // =============================
    always #5 clk = ~clk;

    // =============================
    // Stimulus
    // =============================
    initial begin
        clk = 0;
        rst = 1;
        v_in = 0;

        // Initialize parameters
        Ixx = 32'd2;
        Iyy = 32'd3;
        Izz = 32'd5;

        Ixx_inv = 32'd1;
        Iyy_inv = 32'd1;
        Izz_inv = 32'd1;

        wx = 32'd1;
        wy = 32'd1;
        wz = 32'd1;

        torquex = 32'd10;
        torquey = 32'd0;
        torquez = 32'd0;

        dt = 32'd3;

        // =============================
        // Reset sequence
        // =============================
        #20;
        rst = 0;

        // =============================
        // Trigger RK4 computation
        // =============================
        #10;
        v_in = 1;
        #10;
        v_in = 0;

        // =============================
        // Wait for computation
        // =============================
        #500;

        // =============================
        // Second test (optional)
        // =============================
        wx = 32'd3;
        wy = 32'd1;
        wz = 32'd2;

        torquex = 32'd1;
        torquey = 32'd0;
        torquez = 32'd0;

        #20;
        v_in = 1;
        #10;
        v_in = 0;

        #500;

        $finish;
    end

endmodule
