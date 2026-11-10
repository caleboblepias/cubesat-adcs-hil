`timescale 1ns / 1ps

module spi_registers (
    input logic clk,
    input logic rst,

    input logic [7:0] rx_byte,
    input logic       rx_valid,

    output logic [7:0] tx_byte,
    output logic       tx_load,

    // Pulse from spi_slave when CS returns high.
    input logic transaction_end,

    input logic busy,
    input logic ready,

    output logic start_request,

    // ============================================================
    // Hardware plant-state update
    // ============================================================

    input logic state_update,

    input logic signed [31:0] wx_next,
    input logic signed [31:0] wy_next,
    input logic signed [31:0] wz_next,

    // ============================================================
    // Persistent registers
    // ============================================================

    output logic signed [31:0] torque_x,
    output logic signed [31:0] torque_y,
    output logic signed [31:0] torque_z,

    output logic signed [31:0] wx,
    output logic signed [31:0] wy,
    output logic signed [31:0] wz,

    output logic signed [31:0] dt,

    output logic signed [31:0] ixx,
    output logic signed [31:0] iyy,
    output logic signed [31:0] izz,

    output logic signed [31:0] ixx_inv,
    output logic signed [31:0] iyy_inv,
    output logic signed [31:0] izz_inv
);


    // ============================================================
    // Protocol constants
    // ============================================================

    localparam logic [7:0] CMD_WRITE = 8'h01;
    localparam logic [7:0] CMD_READ  = 8'h02;


    // ============================================================
    // Register addresses
    // ============================================================

    localparam logic [7:0] ADDR_STATUS  = 8'h00;
    localparam logic [7:0] ADDR_CONTROL = 8'h01;

    localparam logic [7:0] ADDR_TORQUE_X = 8'h10;
    localparam logic [7:0] ADDR_TORQUE_Y = 8'h14;
    localparam logic [7:0] ADDR_TORQUE_Z = 8'h18;

    localparam logic [7:0] ADDR_WX = 8'h20;
    localparam logic [7:0] ADDR_WY = 8'h24;
    localparam logic [7:0] ADDR_WZ = 8'h28;

    localparam logic [7:0] ADDR_DT = 8'h30;

    localparam logic [7:0] ADDR_IXX = 8'h40;
    localparam logic [7:0] ADDR_IYY = 8'h44;
    localparam logic [7:0] ADDR_IZZ = 8'h48;

    localparam logic [7:0] ADDR_IXX_INV = 8'h50;
    localparam logic [7:0] ADDR_IYY_INV = 8'h54;
    localparam logic [7:0] ADDR_IZZ_INV = 8'h58;


    // ============================================================
    // Protocol FSM
    // ============================================================

    typedef enum logic [3:0] {
        WAIT_CMD,

        WRITE_ADDR,
        WRITE_DATA_3,
        WRITE_DATA_2,
        WRITE_DATA_1,
        WRITE_DATA_0,

        READ_ADDR,
        READ_DATA_3,
        READ_DATA_2,
        READ_DATA_1,
        READ_DATA_0
    } state_t;

    state_t state;


    logic [7:0] current_addr;

    logic [31:0] write_data;
    logic [31:0] read_snapshot;


    // ============================================================
    // Register read mux
    // ============================================================

    function automatic logic [31:0] read_register (
        input logic [7:0] addr
    );

        begin

            case (addr)

                ADDR_STATUS:
                    read_register = {
                        30'b0,
                        ready,
                        busy
                    };

                // CONTROL is command-only.
                ADDR_CONTROL:
                    read_register = 32'h00000000;

                ADDR_TORQUE_X:
                    read_register = torque_x;

                ADDR_TORQUE_Y:
                    read_register = torque_y;

                ADDR_TORQUE_Z:
                    read_register = torque_z;

                ADDR_WX:
                    read_register = wx;

                ADDR_WY:
                    read_register = wy;

                ADDR_WZ:
                    read_register = wz;

               ADDR_DT:
                    read_register = dt;

                ADDR_IXX:
                    read_register = ixx;

                ADDR_IYY:
                    read_register = iyy;

                ADDR_IZZ:
                    read_register = izz;

                ADDR_IXX_INV:
                    read_register = ixx_inv;

                ADDR_IYY_INV:
                    read_register = iyy_inv;

                ADDR_IZZ_INV:
                    read_register = izz_inv;

                default:
                    read_register = 32'h00000000;

            endcase

        end

    endfunction


    // ============================================================
    // Protocol engine
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            state <= WAIT_CMD;

            current_addr <= 8'h00;

            write_data    <= 32'h00000000;
            read_snapshot <= 32'h00000000;

            tx_byte <= 8'h00;
            tx_load <= 1'b0;

            start_request <= 1'b0;

            torque_x <= '0;
            torque_y <= '0;
            torque_z <= '0;

            wx <= '0;
            wy <= '0;
            wz <= '0;

            dt <= '0;

            ixx <= '0;
            iyy <= '0;
            izz <= '0;

            ixx_inv <= '0;
            iyy_inv <= '0;
            izz_inv <= '0;

        end else begin

            // One-clock pulses.
            tx_load       <= 1'b0;
            start_request <= 1'b0;


            // ====================================================
            // Hardware state update
            //
            // All three state components are committed atomically.
            // ====================================================

            if (state_update) begin

                wx <= wx_next;
                wy <= wy_next;
                wz <= wz_next;

            end


            // ====================================================
            // Transaction boundary
            //
            // If CS rises before a command is complete, discard
            // all partial protocol state. No write is committed.
            // ====================================================

            if (transaction_end) begin

                state <= WAIT_CMD;

                current_addr <= 8'h00;
                write_data   <= 32'h00000000;

            end else if (rx_valid) begin

                case (state)

                    // ============================================
                    // Command byte
                    // ============================================

                    WAIT_CMD: begin

                        if (rx_byte == CMD_WRITE)
                            state <= WRITE_ADDR;

                        else if (rx_byte == CMD_READ)
                            state <= READ_ADDR;

                        else
                            state <= WAIT_CMD;

                    end


                    // ============================================
                    // WRITE
                    // ============================================

                    WRITE_ADDR: begin

                        current_addr <= rx_byte;
                        state <= WRITE_DATA_3;

                    end


                    WRITE_DATA_3: begin

                        write_data[31:24] <= rx_byte;
                        state <= WRITE_DATA_2;

                    end


                    WRITE_DATA_2: begin

                        write_data[23:16] <= rx_byte;
                        state <= WRITE_DATA_1;

                    end


                    WRITE_DATA_1: begin

                        write_data[15:8] <= rx_byte;
                        state <= WRITE_DATA_0;

                    end


                    WRITE_DATA_0: begin

                        case (current_addr)

                            // STATUS is read-only.
                            ADDR_STATUS: begin
                            end


                            // CONTROL is command-only.
                            ADDR_CONTROL: begin

                                if (rx_byte[0])
                                    start_request <= 1'b1;

                            end


                            ADDR_TORQUE_X:
                                torque_x <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_TORQUE_Y:
                                torque_y <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_TORQUE_Z:
                                torque_z <= {
                                    write_data[31:8],
                                    rx_byte
                                };


                            // Hardware state update wins if an SPI
                            // state write completes on the same edge.
                            ADDR_WX: begin

                                if (!state_update)
                                    wx <= {
                                        write_data[31:8],
                                        rx_byte
                                    };

                            end


                            ADDR_WY: begin

                                if (!state_update)
                                    wy <= {
                                        write_data[31:8],
                                        rx_byte
                                    };

                            end


                            ADDR_WZ: begin

                                if (!state_update)
                                    wz <= {
                                        write_data[31:8],
                                        rx_byte
                                    };

                            end


                            ADDR_DT:
                                dt <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_IXX:
                                ixx <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_IYY:
                                iyy <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_IZZ:
                                izz <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_IXX_INV:
                                ixx_inv <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_IYY_INV:
                                iyy_inv <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            ADDR_IZZ_INV:
                                izz_inv <= {
                                    write_data[31:8],
                                    rx_byte
                                };

                            default: begin
                            end

                        endcase

                        state <= WAIT_CMD;

                    end


                    // ============================================
                    // READ
                    // ============================================

                    READ_ADDR: begin

                        current_addr <= rx_byte;

                        read_snapshot <= read_register(rx_byte);

                        // Vivado 2021.1 is happier with a shift here
                        // than indexing a function return directly.
                        tx_byte <= read_register(rx_byte) >> 24;
                        tx_load <= 1'b1;

                        state <= READ_DATA_3;

                    end


                    READ_DATA_3: begin

                        tx_byte <= read_snapshot[23:16];
                        tx_load <= 1'b1;

                        state <= READ_DATA_2;

                    end


                    READ_DATA_2: begin

                        tx_byte <= read_snapshot[15:8];
                        tx_load <= 1'b1;

                        state <= READ_DATA_1;

                    end


                    READ_DATA_1: begin

                        tx_byte <= read_snapshot[7:0];
                        tx_load <= 1'b1;

                        state <= READ_DATA_0;

                    end


                    READ_DATA_0: begin

                        state <= WAIT_CMD;

                    end


                    default: begin

                        state <= WAIT_CMD;

                    end

                endcase

            end

        end

    end

endmodule 
