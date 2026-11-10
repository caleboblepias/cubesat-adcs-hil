`timescale 1ns / 1ps

module rigid_body_dynamics (
    input  logic clk,
    input  logic rst,

    input  logic start,
    output logic done,

    // Angular velocity, Q16.16
    input  logic signed [31:0] wx,
    input  logic signed [31:0] wy,
    input  logic signed [31:0] wz,

    // Applied body torque, Q16.16
    input  logic signed [31:0] torque_x,
    input  logic signed [31:0] torque_y,
    input  logic signed [31:0] torque_z,

    // Principal moments of inertia, Q16.16
    input  logic signed [31:0] Ixx,
    input  logic signed [31:0] Iyy,
    input  logic signed [31:0] Izz,

    // Inverse principal moments, Q16.16
    input  logic signed [31:0] Ixx_inv,
    input  logic signed [31:0] Iyy_inv,
    input  logic signed [31:0] Izz_inv,

    // Angular acceleration, Q16.16
    output logic signed [31:0] wx_dot,
    output logic signed [31:0] wy_dot,
    output logic signed [31:0] wz_dot
);

    /*
     * For diagonal inertia:
     *
     * wx_dot = Ixx_inv *
     *          (torque_x - (Izz - Iyy)*wy*wz)
     *
     * wy_dot = Iyy_inv *
     *          (torque_y - (Ixx - Izz)*wz*wx)
     *
     * wz_dot = Izz_inv *
     *          (torque_z - (Iyy - Ixx)*wx*wy)
     */


    // ============================================================
    // State machine
    // ============================================================

    typedef enum logic [3:0] {
        IDLE,

        CROSS_PRODUCTS_START,
        CROSS_PRODUCTS_WAIT,

        CROSS_SCALE_START,
        CROSS_SCALE_WAIT,

        TORQUE_DIFF,

        ACCELERATION_START,
        ACCELERATION_WAIT,

        DONE_STATE
    } state_t;

    state_t state;


    // ============================================================
    // Latched inputs
    // ============================================================

    logic signed [31:0] wx_reg;
    logic signed [31:0] wy_reg;
    logic signed [31:0] wz_reg;

    logic signed [31:0] tx_reg;
    logic signed [31:0] ty_reg;
    logic signed [31:0] tz_reg;

    logic signed [31:0] Ixx_reg;
    logic signed [31:0] Iyy_reg;
    logic signed [31:0] Izz_reg;

    logic signed [31:0] Ixx_inv_reg;
    logic signed [31:0] Iyy_inv_reg;
    logic signed [31:0] Izz_inv_reg;


    // ============================================================
    // Intermediate values
    // ============================================================

    logic signed [31:0] wy_wz_reg;
    logic signed [31:0] wz_wx_reg;
    logic signed [31:0] wx_wy_reg;

    logic signed [31:0] cross_x;
    logic signed [31:0] cross_y;
    logic signed [31:0] cross_z;

    logic signed [31:0] diff_x;
    logic signed [31:0] diff_y;
    logic signed [31:0] diff_z;


    // ============================================================
    // Three parallel sequential multipliers
    // ============================================================

    logic mul_start;

    logic signed [31:0] mul_x_a;
    logic signed [31:0] mul_x_b;
    logic signed [31:0] mul_y_a;
    logic signed [31:0] mul_y_b;
    logic signed [31:0] mul_z_a;
    logic signed [31:0] mul_z_b;

    logic signed [31:0] mul_x_result;
    logic signed [31:0] mul_y_result;
    logic signed [31:0] mul_z_result;

    logic mul_x_done;
    logic mul_y_done;
    logic mul_z_done;


    qmul_sequential mul_x (
        .clk    (clk),
        .rst    (rst),
        .start  (mul_start),
        .done   (mul_x_done),
        .a      (mul_x_a),
        .b      (mul_x_b),
        .result (mul_x_result)
    );

    qmul_sequential mul_y (
        .clk    (clk),
        .rst    (rst),
        .start  (mul_start),
        .done   (mul_y_done),
        .a      (mul_y_a),
        .b      (mul_y_b),
        .result (mul_y_result)
    );

    qmul_sequential mul_z (
        .clk    (clk),
        .rst    (rst),
        .start  (mul_start),
        .done   (mul_z_done),
        .a      (mul_z_a),
        .b      (mul_z_b),
        .result (mul_z_result)
    );


    // ============================================================
    // Multiplier operand selection
    // ===========================================================

    always_comb begin

        mul_start = 1'b0;

        mul_x_a = '0;
        mul_x_b = '0;

        mul_y_a = '0;
        mul_y_b = '0;

        mul_z_a = '0;
        mul_z_b = '0;

        case (state)

            // ----------------------------------------------------
            // wy*wz, wz*wx, wx*wy
            // ----------------------------------------------------

            CROSS_PRODUCTS_START: begin

                mul_start = 1'b1;

                mul_x_a = wy_reg;
                mul_x_b = wz_reg;

                mul_y_a = wz_reg;
                mul_y_b = wx_reg;

                mul_z_a = wx_reg;
                mul_z_b = wy_reg;

            end


            // ----------------------------------------------------
            // Inertia-difference scaling
            // ----------------------------------------------------

            CROSS_SCALE_START: begin

                mul_start = 1'b1;

                mul_x_a = Izz_reg - Iyy_reg;
                mul_x_b = wy_wz_reg;

                mul_y_a = Ixx_reg - Izz_reg;
                mul_y_b = wz_wx_reg;

                mul_z_a = Iyy_reg - Ixx_reg;
                mul_z_b = wx_wy_reg;

            end


            // ----------------------------------------------------
            // Inverse-inertia scaling
            // ----------------------------------------------------

            ACCELERATION_START: begin

                mul_start = 1'b1;

                mul_x_a = Ixx_inv_reg;
                mul_x_b = diff_x;

                mul_y_a = Iyy_inv_reg;
                mul_y_b = diff_y;

                mul_z_a = Izz_inv_reg;
                mul_z_b = diff_z;

            end


            default: begin
                mul_start = 1'b0;
            end

        endcase

    end


    // ============================================================
    // Sequential control
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            state <= IDLE;
            done <= 1'b0;

            wx_reg <= '0;
            wy_reg <= '0;
            wz_reg <= '0;

            tx_reg <= '0;
            ty_reg <= '0;
            tz_reg <= '0;

            Ixx_reg <= '0;
            Iyy_reg <= '0;
            Izz_reg <= '0;

            Ixx_inv_reg <= '0;
            Iyy_inv_reg <= '0;
            Izz_inv_reg <= '0;

            wy_wz_reg <= '0;
            wz_wx_reg <= '0;
            wx_wy_reg <= '0;

            cross_x <= '0;
            cross_y <= '0;
            cross_z <= '0;

            diff_x <= '0;
            diff_y <= '0;
            diff_z <= '0;

            wx_dot <= '0;
            wy_dot <= '0;
            wz_dot <= '0;

        end else begin

            done <= 1'b0;

            case (state)

                // =================================================
                // Capture complete transaction
                // =================================================

                IDLE: begin

                    if (start) begin

                        wx_reg <= wx;
                        wy_reg <= wy;
                        wz_reg <= wz;

                        tx_reg <= torque_x;
                        ty_reg <= torque_y;
                        tz_reg <= torque_z;

                        Ixx_reg <= Ixx;
                        Iyy_reg <= Iyy;
                        Izz_reg <= Izz;

                        Ixx_inv_reg <= Ixx_inv;
                        Iyy_inv_reg <= Iyy_inv;
                        Izz_inv_reg <= Izz_inv;

                        state <= CROSS_PRODUCTS_START;

                    end

                end


                // =================================================
                // Start:
                //
                // wy*wz
                // wz*wx
                // wx*wy
                // =================================================

                CROSS_PRODUCTS_START: begin

                    state <= CROSS_PRODUCTS_WAIT;

                end


                CROSS_PRODUCTS_WAIT: begin

                    if (mul_x_done &&
                        mul_y_done &&
                        mul_z_done) begin

                        wy_wz_reg <= mul_x_result;
                        wz_wx_reg <= mul_y_result;
                        wx_wy_reg <= mul_z_result;

                        state <= CROSS_SCALE_START;

                    end

                end


                // =================================================
                // Start:
                //
                // (Izz-Iyy)*(wy*wz)
                // (Ixx-Izz)*(wz*wx)
                // (Iyy-Ixx)*(wx*wy)
                // =================================================

                CROSS_SCALE_START: begin

                    state <= CROSS_SCALE_WAIT;

                end


                CROSS_SCALE_WAIT: begin

                    if (mul_x_done &&
                        mul_y_done &&
                        mul_z_done) begin

                        cross_x <= mul_x_result;
                        cross_y <= mul_y_result;
                        cross_z <= mul_z_result;

                        state <= TORQUE_DIFF;

                    end

                end


                // =================================================
                // tau - w x (I w)
                // =================================================

                TORQUE_DIFF: begin

                    diff_x <= tx_reg - cross_x;
                    diff_y <= ty_reg - cross_y;
                    diff_z <= tz_reg - cross_z;

                    state <= ACCELERATION_START;

                end


                // =================================================
                // Start inverse-inertia multiplication
                // =================================================

                ACCELERATION_START: begin

                    state <= ACCELERATION_WAIT;

                end


                ACCELERATION_WAIT: begin

                    if (mul_x_done &&
                        mul_y_done &&
                        mul_z_done) begin

                        wx_dot <= mul_x_result;
                        wy_dot <= mul_y_result;
                        wz_dot <= mul_z_result;

                        state <= DONE_STATE;

                    end

                end


                // =================================================
                // One-cycle completion pulse
                // =================================================

                DONE_STATE: begin

                    done <= 1'b1;
                    state <= IDLE;

                end


                default: begin
                    state <= IDLE;
                end

            endcase

        end

    end

endmodule
