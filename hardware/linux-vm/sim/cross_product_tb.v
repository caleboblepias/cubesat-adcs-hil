`timescale 1ns / 1ps

module cross_product_tb;

reg  [31:0] wx, wy, wz;
reg  [31:0] vx, vy, vz;

wire [31:0] cx, cy, cz;

// Instantiate your module
cross_product uut (
    .wx(wx), .wy(wy), .wz(wz),
    .vx(vx), .vy(vy), .vz(vz),
    .cx(cx), .cy(cy), .cz(cz)
);

initial begin
    $dumpfile("wave.vcd");
    $dumpvars(1, cross_product_tb);

    // Test case 1
    wx = 1; wy = 0; wz = 0;
    vx = 0; vy = 1; vz = 0;

    #1;
    $display("Test1: cx=%0d cy=%0d cz=%0d", cx, cy, cz);

    #9

    // Test case 2
    wx = 0; wy = 1; wz = 0;
    vx = 0; vy = 0; vz = 1;

    #1;
    $display("Test2: cx=%0d cy=%0d cz=%0d", cx, cy, cz);

    #9;

    // Test case 3
    wx = 1; wy = 2; wz = 3;
    vx = 4; vy = 5; vz = 6;

    #1;
    $display("Test3: cx=%0d cy=%0d cz=%0d", cx, cy, cz);

    #9;


    #20;
    $finish;
end

endmodule
