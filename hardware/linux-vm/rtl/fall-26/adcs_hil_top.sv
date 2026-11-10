`timescale 1ns / 1ps

module adcs_hil_top (
    input  logic clk,
    input  logic rst,

    // ============================================================
    // External SPI interface
    // ============================================================

    input  logic spi_cs_n,
    input  logic spi_sclk,
    input  logic spi_mosi,
    output logic spi_miso
);

    // ============================================================
    // SPI slave <-> register interface
    // ============================================================

    logic [7:0] rx_byte;
    logic       rx_valid;

    logic [7:0] tx_byte;
    logic       tx_load;

    logic       transaction_end;


    // ============================================================
    // Register outputs / plant inputs
    // ============================================================

    logic signed [31:0] torque_x;
    logic signed [31:0] torque_y;
    logic signed [31:0] torque_z;

    logic signed [31:0] wx;
    logic signed [31:0] wy;
    logic signed [31:0] wz;

    logic signed [31:0] dt;

    logic signed [31:0] ixx;
    logic signed [31:0] iyy;
    logic signed [31:0] izz;

    logic signed [31:0] ixx_inv;
    logic signed [31:0] iyy_inv;
    logic signed [31:0] izz_inv;


    // ============================================================
    // Register <-> plant controller interface
    // ============================================================

    logic start_request;

    logic busy;
    logic ready;

    logic state_update;

    logic signed [31:0] wx_next;
    logic signed [31:0] wy_next;
    logic signed [31:0] wz_next;


    // ============================================================
    // SPI physical layer
    // ============================================================

    spi_slave spi_slave_inst (
        .clk             (clk),
        .rst             (rst),

        .spi_cs_n        (spi_cs_n),
        .spi_sclk        (spi_sclk),
        .spi_mosi        (spi_mosi),
        .spi_miso        (spi_miso),

        .rx_byte         (rx_byte),
        .rx_valid        (rx_valid),

        .tx_byte         (tx_byte),
        .tx_load         (tx_load),

        .transaction_end (transaction_end)
    );


    // ============================================================
    // SPI protocol / register map
    // ============================================================

    spi_registers spi_registers_inst (
        .clk             (clk),
        .rst             (rst),

        .rx_byte         (rx_byte),
        .rx_valid        (rx_valid),

        .tx_byte         (tx_byte),
        .tx_load         (tx_load),

        .transaction_end (transaction_end),

        .busy            (busy),
        .ready           (ready),

        .start_request   (start_request),

        .state_update    (state_update),

        .wx_next         (wx_next),
        .wy_next         (wy_next),
        .wz_next         (wz_next),

        .torque_x        (torque_x),
        .torque_y        (torque_y),
        .torque_z        (torque_z),

        .wx              (wx),
        .wy              (wy),
        .wz              (wz),

        .dt              (dt),

        .ixx             (ixx),
        .iyy             (iyy),
        .izz             (izz),

        .ixx_inv         (ixx_inv),
        .iyy_inv         (iyy_inv),
        .izz_inv         (izz_inv)
    );


    // ============================================================
    // Plant controller
    // ============================================================

    plant_controller plant_controller_inst (
        .clk             (clk),
        .rst             (rst),

        .start_request   (start_request),

        .wx              (wx),
        .wy              (wy),
        .wz              (wz),

        .torque_x        (torque_x),
        .torque_y        (torque_y),
        .torque_z        (torque_z),

        .dt              (dt),

        .ixx             (ixx),
        .iyy             (iyy),
        .izz             (izz),

        .ixx_inv        (ixx_inv),
        .iyy_inv         (iyy_inv),
        .izz_inv         (izz_inv),

        .busy            (busy),
        .ready           (ready),

        .state_update    (state_update),

        .wx_next         (wx_next),
        .wy_next         (wy_next),
        .wz_next         (wz_next)
    );

endmodule 
