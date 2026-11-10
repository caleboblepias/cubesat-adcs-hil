`timescale 1ns / 1ps

module spi_slave (
    input  logic       clk,
    input  logic       rst,

    input  logic       spi_cs_n,
    input  logic       spi_sclk,
    input  logic       spi_mosi,
    output logic       spi_miso,

    output logic [7:0] rx_byte,
    output logic       rx_valid,

    input  logic [7:0] tx_byte,
    input  logic       tx_load,

    // One clk pulse when an SPI transaction ends.
    output logic       transaction_end
);
    
    // ============================================================
    // Synchronizers
    // ============================================================

    (* ASYNC_REG = "TRUE" *) logic cs_meta;
    (* ASYNC_REG = "TRUE" *) logic cs_sync;
    logic cs_prev;

    (* ASYNC_REG = "TRUE" *) logic sclk_meta;
    (* ASYNC_REG = "TRUE" *) logic sclk_sync;
    logic sclk_prev;

    (* ASYNC_REG = "TRUE" *) logic mosi_meta;
    (* ASYNC_REG = "TRUE" *) logic mosi_sync;


    // ============================================================
    // SPI state
    // ============================================================

    logic [7:0] rx_shift;
    logic [7:0] tx_shift;

    logic [2:0] bit_count;

    // Indicates that a TX byte was loaded while SCLK was high.
    //
    // In that case, the next synchronized falling edge belongs to
    // the previous SPI byte and must not shift the newly loaded byte.
    logic tx_fresh;


    // ============================================================
    // Edge detection
    // ============================================================

    wire cs_falling =
        (cs_prev == 1'b1) &&
        (cs_sync == 1'b0);

    wire cs_rising =
        (cs_prev == 1'b0) &&
        (cs_sync == 1'b1);

    wire sclk_rising =
        (sclk_prev == 1'b0) &&
        (sclk_sync == 1'b1);

    wire sclk_falling =
        (sclk_prev == 1'b1) &&
        (sclk_sync == 1'b0);


    // ============================================================
    // Mode-0 MISO
    //
    // Data is presented before the master's rising sample edge.
    // ============================================================


	assign spi_miso = (!spi_cs_n) ? tx_shift[7] : 1'b0;

    // ============================================================
    // Synchronize asynchronous SPI inputs
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            cs_meta   <= 1'b1;
            cs_sync   <= 1'b1;
            cs_prev   <= 1'b1;

            sclk_meta <= 1'b0;
            sclk_sync <= 1'b0;
            sclk_prev <= 1'b0;

            mosi_meta <= 1'b0;
            mosi_sync <= 1'b0;

        end else begin

            cs_meta <= spi_cs_n;
            cs_sync <= cs_meta;
            cs_prev <= cs_sync;

            sclk_meta <= spi_sclk;
            sclk_sync <= sclk_meta;
            sclk_prev <= sclk_sync;

            mosi_meta <= spi_mosi;
            mosi_sync <= mosi_meta;

        end

    end


    // ============================================================
    // SPI byte engine
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            rx_shift <= 8'h00;
            tx_shift <= 8'h00;

            rx_byte <= 8'h00;

            bit_count <= 3'd0;

            rx_valid        <= 1'b0;
            transaction_end <= 1'b0;

            tx_fresh <= 1'b0;

        end else begin

            // One-clock pulses.
            rx_valid        <= 1'b0;
            transaction_end <= 1'b0;


            // ----------------------------------------------------
            // End of SPI transaction
            // ----------------------------------------------------

            if (cs_rising) begin

                transaction_end <= 1'b1;

                bit_count <= 3'd0;
                rx_shift  <= 8'h00;

                tx_fresh <= 1'b0;

            end


            // ----------------------------------------------------
            // Beginning of SPI transaction
            // ----------------------------------------------------

            if (cs_falling) begin

                bit_count <= 3'd0;
                rx_shift  <= 8'h00;

                tx_shift <= tx_byte;

                // The first byte is already loaded before its first
                // active SCLK edge, so no falling-edge suppression
                // is required.
                tx_fresh <= 1'b0;

            end


            // ----------------------------------------------------
            // Protocol layer loads next transmit byte
            //
            // If the byte is loaded while synchronized SCLK is high,
            // the upcoming falling edge still terminates the previous
            // byte. Preserve the newly loaded byte across that edge.
            //
            // If loaded while SCLK is low, the next falling edge is
            // part of the newly loaded byte and should shift normally.
            // ----------------------------------------------------

            if (tx_load) begin

                tx_shift <= tx_byte;
                tx_fresh <= sclk_sync;

            end


            // ----------------------------------------------------
            // Sample MOSI on rising SCLK
            // ----------------------------------------------------

            if ((!cs_sync) && sclk_rising) begin

                rx_shift <= {
                    rx_shift[6:0],
                    mosi_sync
                };

                if (bit_count == 3'd7) begin

                    rx_byte <= {
                        rx_shift[6:0],
                        mosi_sync
                    };

                    rx_valid <= 1'b1;

                    bit_count <= 3'd0;

                end else begin

                    bit_count <= bit_count + 1'b1;

                end

            end


            // ----------------------------------------------------
            // Shift MISO on falling SCLK
            // ----------------------------------------------------

            if ((!cs_sync) && sclk_falling) begin

                if (tx_load) begin

                    // tx_load and the falling edge occurred on the
                    // same FPGA clock. The newly loaded byte wins and
                    // must not be shifted.
                    tx_fresh <= 1'b0;

                end else if (tx_fresh) begin

                    // The byte was loaded while SCLK was high.
                    // This falling edge terminates the previous byte,
                    // so preserve the freshly loaded byte.
                    tx_fresh <= 1'b0;

                end else begin

                    // Normal Mode-0 falling edge: advance to the next
                    // transmit bit.
                    tx_shift <= {
                        tx_shift[6:0],
                        1'b0
                    };

                end

            end

        end

    end

endmodule
