`timescale 1ns / 1ps

module adcs_hil_hw_top (
    input  logic clk_125mhz,
    input  logic reset_sw,

    input  logic spi_cs_n,
    input  logic spi_sclk,
    input  logic spi_mosi,
    output logic spi_miso
);

    // ============================================================
    // Reset synchronizer
    //
    // Asynchronous assertion, synchronous deassertion.
    // ============================================================

    (* ASYNC_REG = "TRUE" *) logic [1:0] reset_sync;

    always_ff @(posedge clk_125mhz or posedge reset_sw) begin

        if (reset_sw) begin
            reset_sync <= 2'b11;
        end else begin
            reset_sync <= {reset_sync[0], 1'b0};
        end

    end

    logic rst;

    assign rst = reset_sync[1];


    // ============================================================
    // ADCS HIL core
    // ============================================================

    adcs_hil_top adcs_hil_inst (
        .clk      (clk_125mhz),
        .rst      (rst),

        .spi_cs_n (spi_cs_n),
        .spi_sclk (spi_sclk),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );

endmodule
