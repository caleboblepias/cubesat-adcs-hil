`timescale 1ns / 1ps

module plant_controller (
    input logic clk,
    input logic rst,

    // ============================================================
    // Command from SPI register file
    // ============================================================

    input logic start_request,

    // ============================================================
    // Current persistent plant state
    // ============================================================

    input logic signed [31:0] wx,
    input logic signed [31:0] wy,
    input logic signed [31:0] wz,

    // ============================================================
    // Applied body torque
    // ============================================================

    input logic signed [31:0] torque_x,
    input logic signed [31:0] torque_y,
    input logic signed [31:0] torque_z,

    // ============================================================
    // Integration timestep
    // ============================================================

    input logic signed [31:0] dt,

    // ============================================================
    // Principal moments of inertia
    // ============================================================

    input logic signed [31:0] ixx,
    input logic signed [31:0] iyy,
    input logic signed [31:0] izz,

    // ============================================================
    // Inverse principal moments of inertia
    // ============================================================

    input logic signed [31:0] ixx_inv,
    input logic signed [31:0] iyy_inv,
    input logic signed [31:0] izz_inv,

    // ============================================================
    // Status to SPI register file
    // ============================================================

    output logic busy,
    output logic ready,

    // ============================================================
    // Atomic plant-state update to SPI register file
    // ============================================================

    output logic state_update,

    output logic signed [31:0] wx_next,
    output logic signed [31:0] wy_next,
    output logic signed [31:0] wz_next
);


    // ============================================================
    // RK4 interface
    // ============================================================

    logic rk4_start;
    logic rk4_done;

    logic signed [31:0] rk4_wx_next;
    logic signed [31:0] rk4_wy_next;
    logic signed [31:0] rk4_wz_next;


    // ============================================================
    // Controller state
    // ============================================================

    typedef enum logic [1:0] {
        IDLE,
        RUNNING,
        COMMIT
    } state_t;

    state_t state;


    // ============================================================
    // RK4 numerical integrator
    //
    // IMPORTANT:
    // RK4's inertia ports use uppercase I:
    //
    // Ixx, Iyy, Izz
    // Ixx_inv, Iyy_inv, Izz_inv
    //
    // The plant_controller external interface uses lowercase names.
    // ============================================================

    rk4 rk4_inst (
        .clk       (clk),
        .rst       (rst),
        .start     (rk4_start),

        .wx        (wx),
        .wy        (wy),
        .wz        (wz),

        .torque_x  (torque_x),
        .torque_y  (torque_y),
        .torque_z  (torque_z),

        .dt        (dt),

        .Ixx       (ixx),
        .Iyy       (iyy),
        .Izz       (izz),

        .Ixx_inv   (ixx_inv),
        .Iyy_inv   (iyy_inv),
        .Izz_inv   (izz_inv),

        .wx_next   (rk4_wx_next),
        .wy_next   (rk4_wy_next),
        .wz_next   (rk4_wz_next),

        .done      (rk4_done)
    );


    // ============================================================
    // Controller FSM
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            state <= IDLE;

            busy  <= 1'b0;
            ready <= 1'b0;

           rk4_start    <= 1'b0;
            state_update <= 1'b0;

            wx_next <= '0;
            wy_next <= '0;
            wz_next <= '0;

        end else begin

            // Pulse outputs default low.
            rk4_start    <= 1'b0;
            state_update <= 1'b0;


            case (state)

                // =================================================
                // Waiting for a new integration request
                // =================================================

                IDLE: begin

                    busy <= 1'b0;

                    if (start_request) begin

                        // A new calculation invalidates the
                        // previously READY result.
                        ready <= 1'b0;
                        busy  <= 1'b1;

                        // RK4 latches its inputs on this pulse.
                        rk4_start <= 1'b1;

                        state <= RUNNING;

                    end

                end


                // =================================================
                // RK4 calculation in progress
                //
                // START requests received in this state are ignored.
                // =================================================

                RUNNING: begin

                    busy <= 1'b1;

                    if (rk4_done) begin

                        // Capture the completed RK4 result before
                        // requesting the persistent-state commit.
                        wx_next <= rk4_wx_next;
                        wy_next <= rk4_wy_next;
                        wz_next <= rk4_wz_next;

                        state <= COMMIT;

                    end

                end


                // =================================================
                // Commit new persistent plant state
                //
                // state_update is asserted for exactly one clock.
                // spi_registers consumes wx_next/wy_next/wz_next
                // and atomically updates WX/WY/WZ.
                // =================================================

                COMMIT: begin

                    state_update <= 1'b1;

                    busy  <= 1'b0;
                    ready <= 1'b1;

                    state <= IDLE;

                end


                // =================================================
                // Recovery
                // =================================================

                default: begin

                    state <= IDLE;

                    busy  <= 1'b0;
                    ready <= 1'b0;

                    rk4_start    <= 1'b0;
                    state_update <= 1'b0;

                end

            endcase

        end

    end

endmodule 
