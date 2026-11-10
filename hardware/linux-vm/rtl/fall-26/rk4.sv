`timescale 1ns / 1ps

module rk4 (
    input  logic clk,
    input  logic rst,

    input  logic start,
    output logic done,

    // Initial angular velocity, Q16.16
    input logic signed [31:0] wx,
    input logic signed [31:0] wy,
    input logic signed [31:0] wz,

    // Applied body torque, Q16.16
    input logic signed [31:0] torque_x,
    input logic signed [31:0] torque_y,
    input logic signed [31:0] torque_z,

    // Integration timestep, Q16.16
    input logic signed [31:0] dt,

    // Principal moments of inertia, Q16.16
    input logic signed [31:0] Ixx,
    input logic signed [31:0] Iyy,
    input logic signed [31:0] Izz,

    // Inverse principal moments, Q16.16
    input logic signed [31:0] Ixx_inv,
    input logic signed [31:0] Iyy_inv,
    input logic signed [31:0] Izz_inv,

    // Integrated angular velocity, Q16.16
    output logic signed [31:0] wx_next,
    output logic signed [31:0] wy_next,
    output logic signed [31:0] wz_next
);


    // ============================================================
    // State machine
    // ============================================================

    typedef enum logic [4:0] {
        IDLE,

        START_K1,
        WAIT_K1,

        SCALE_X2_START,
        SCALE_X2_WAIT,
        CALC_X2,

        START_K2,
        WAIT_K2,

        SCALE_X3_START,
        SCALE_X3_WAIT,
        CALC_X3,

        START_K3,
        WAIT_K3,

        SCALE_X4_START,
        SCALE_X4_WAIT,
        CALC_X4,

        START_K4,
        WAIT_K4,

        FINAL_WEIGHT,
        DIV_START,
        DIV_RUN,
        DIV_FINISH,

        FINAL_SCALE_START,
        FINAL_SCALE_WAIT,
        FINAL_UPDATE,

        DONE_STATE
    } state_t;

    state_t state;


    // ============================================================
    // Latched transaction inputs
    // ============================================================

    logic signed [31:0] wx_reg;
    logic signed [31:0] wy_reg;
    logic signed [31:0] wz_reg;

    logic signed [31:0] tx_reg;
    logic signed [31:0] ty_reg;
    logic signed [31:0] tz_reg;

    logic signed [31:0] dt_reg;

    logic signed [31:0] Ixx_reg;
    logic signed [31:0] Iyy_reg;
    logic signed [31:0] Izz_reg;

    logic signed [31:0] Ixx_inv_reg;
    logic signed [31:0] Iyy_inv_reg;
    logic signed [31:0] Izz_inv_reg;


    // ============================================================
    // RK4 intermediate states
    // ============================================================

    logic signed [31:0] x2_x;
    logic signed [31:0] x2_y;
    logic signed [31:0] x2_z;

    logic signed [31:0] x3_x;
    logic signed [31:0] x3_y;
    logic signed [31:0] x3_z;

    logic signed [31:0] x4_x;
    logic signed [31:0] x4_y;
    logic signed [31:0] x4_z;


    // ============================================================
    // RK4 derivatives
    // ============================================================

    logic signed [31:0] k1_x;
    logic signed [31:0] k1_y;
    logic signed [31:0] k1_z;

    logic signed [31:0] k2_x;
    logic signed [31:0] k2_y;
    logic signed [31:0] k2_z;

    logic signed [31:0] k3_x;
    logic signed [31:0] k3_y;
    logic signed [31:0] k3_z;

    logic signed [31:0] k4_x;
    logic signed [31:0] k4_y;
    logic signed [31:0] k4_z;


    // ============================================================
    // dt*k intermediate values
    // ============================================================

    logic signed [31:0] dt_k_x_reg;
    logic signed [31:0] dt_k_y_reg;
    logic signed [31:0] dt_k_z_reg;


    // ============================================================
    // Final RK4 calculation
    // ============================================================

    logic signed [34:0] weighted_x_reg;
    logic signed [34:0] weighted_y_reg;
    logic signed [34:0] weighted_z_reg;

    logic signed [31:0] average_x_reg;
    logic signed [31:0] average_y_reg;
    logic signed [31:0] average_z_reg;

    logic signed [31:0] delta_x_reg;
    logic signed [31:0] delta_y_reg;
    logic signed [31:0] delta_z_reg;


    // ============================================================
    // Iterative divide-by-6
    // ============================================================

    logic [34:0] div_x_dividend;
    logic [34:0] div_y_dividend;
    logic [34:0] div_z_dividend;

    logic [34:0] div_x_quotient;
    logic [34:0] div_y_quotient;
    logic [34:0] div_z_quotient;

    logic [2:0] div_x_remainder;
    logic [2:0] div_y_remainder;
    logic [2:0] div_z_remainder;

    logic div_x_negative;
    logic div_y_negative;
    logic div_z_negative;

    logic [5:0] div_count;

    logic [3:0] div_x_trial;
    logic [3:0] div_y_trial;
    logic [3:0] div_z_trial;


    // ============================================================
    // Rigid-body dynamics interface
    // ============================================================

    logic dynamics_start;
    logic dynamics_done;

    logic signed [31:0] dynamics_wx;
    logic signed [31:0] dynamics_wy;
    logic signed [31:0] dynamics_wz;

    logic signed [31:0] dynamics_wx_dot;
    logic signed [31:0] dynamics_wy_dot;
    logic signed [31:0] dynamics_wz_dot;


    // ============================================================
    // Three parallel RK4 multipliers
    // ============================================================

    logic rk_mul_start;

    logic signed [31:0] rk_mul_x_a;
    logic signed [31:0] rk_mul_x_b;
    logic signed [31:0] rk_mul_y_a;
    logic signed [31:0] rk_mul_y_b;
    logic signed [31:0] rk_mul_z_a;
    logic signed [31:0] rk_mul_z_b;

    logic signed [31:0] rk_mul_x_result;
    logic signed [31:0] rk_mul_y_result;
    logic signed [31:0] rk_mul_z_result;

    logic rk_mul_x_done;
    logic rk_mul_y_done;
    logic rk_mul_z_done;


    qmul_sequential rk_mul_x (
        .clk    (clk),
        .rst    (rst),
        .start  (rk_mul_start),
        .done   (rk_mul_x_done),
        .a      (rk_mul_x_a),
        .b      (rk_mul_x_b),
        .result (rk_mul_x_result)
    );

    qmul_sequential rk_mul_y (
        .clk    (clk),
        .rst    (rst),
        .start  (rk_mul_start),
        .done   (rk_mul_y_done),
        .a      (rk_mul_y_a),
        .b      (rk_mul_y_b),
        .result (rk_mul_y_result)
    );

    qmul_sequential rk_mul_z (
        .clk    (clk),
        .rst    (rst),
        .start  (rk_mul_start),
        .done   (rk_mul_z_done),
        .a      (rk_mul_z_a),
        .b      (rk_mul_z_b),
        .result (rk_mul_z_result)
    );


    // ============================================================
    // Rigid-body dynamics
    // ============================================================

    rigid_body_dynamics dynamics (
        .clk      (clk),
        .rst      (rst),

        .start    (dynamics_start),
        .done     (dynamics_done),

        .wx       (dynamics_wx),
        .wy       (dynamics_wy),
        .wz       (dynamics_wz),

        .torque_x (tx_reg),
        .torque_y (ty_reg),
        .torque_z (tz_reg),

        .Ixx      (Ixx_reg),
        .Iyy      (Iyy_reg),
        .Izz      (Izz_reg),

        .Ixx_inv  (Ixx_inv_reg),
        .Iyy_inv  (Iyy_inv_reg),
        .Izz_inv  (Izz_inv_reg),

        .wx_dot   (dynamics_wx_dot),
        .wy_dot   (dynamics_wy_dot),
        .wz_dot   (dynamics_wz_dot)
    );


    // ============================================================
    // Dynamics state selection
    // ============================================================

    always_comb begin

        dynamics_wx = wx_reg;
        dynamics_wy = wy_reg;
        dynamics_wz = wz_reg;

        case (state)

            START_K2,
            WAIT_K2: begin
                dynamics_wx = x2_x;
                dynamics_wy = x2_y;
                dynamics_wz = x2_z;
            end

            START_K3,
            WAIT_K3: begin
                dynamics_wx = x3_x;
                dynamics_wy = x3_y;
                dynamics_wz = x3_z;
            end

            START_K4,
            WAIT_K4: begin
                dynamics_wx = x4_x;
                dynamics_wy = x4_y;
                dynamics_wz = x4_z;
            end

            default: begin
                dynamics_wx = wx_reg;
                dynamics_wy = wy_reg;
                dynamics_wz = wz_reg;
            end

        endcase

    end


    // ============================================================
    // Dynamics start pulse
    // ============================================================

    always_comb begin

        case (state)

            START_K1,
            START_K2,
            START_K3,
            START_K4:
                dynamics_start = 1'b1;

            default:
                dynamics_start = 1'b0;

        endcase

    end


    // ============================================================
    // RK4 multiplier operand selection
    // ============================================================

    always_comb begin

        rk_mul_start = 1'b0;

        rk_mul_x_a = '0;
        rk_mul_x_b = '0;

        rk_mul_y_a = '0;
        rk_mul_y_b = '0;

        rk_mul_z_a = '0;
        rk_mul_z_b = '0;

        case (state)

            SCALE_X2_START: begin

                rk_mul_start = 1'b1;

                rk_mul_x_a = dt_reg;
                rk_mul_x_b = k1_x;

                rk_mul_y_a = dt_reg;
                rk_mul_y_b = k1_y;

                rk_mul_z_a = dt_reg;
                rk_mul_z_b = k1_z;

            end


            SCALE_X3_START: begin

                rk_mul_start = 1'b1;

                rk_mul_x_a = dt_reg;
                rk_mul_x_b = k2_x;

                rk_mul_y_a = dt_reg;
                rk_mul_y_b = k2_y;

                rk_mul_z_a = dt_reg;
                rk_mul_z_b = k2_z;

            end


            SCALE_X4_START: begin

                rk_mul_start = 1'b1;

                rk_mul_x_a = dt_reg;
                rk_mul_x_b = k3_x;

                rk_mul_y_a = dt_reg;
                rk_mul_y_b = k3_y;

                rk_mul_z_a = dt_reg;
                rk_mul_z_b = k3_z;

            end


            FINAL_SCALE_START: begin

                rk_mul_start = 1'b1;

                rk_mul_x_a = dt_reg;
                rk_mul_x_b = average_x_reg;

                rk_mul_y_a = dt_reg;
                rk_mul_y_b = average_y_reg;

                rk_mul_z_a = dt_reg;
                rk_mul_z_b = average_z_reg;

            end


            default: begin
                rk_mul_start = 1'b0;
            end

        endcase

    end


    // ============================================================
    // Divider trial remainders
    // ============================================================

    always_comb begin

        div_x_trial = {div_x_remainder, div_x_dividend[34]};
        div_y_trial = {div_y_remainder, div_y_dividend[34]};
        div_z_trial = {div_z_remainder, div_z_dividend[34]};

    end


    // ============================================================
    // RK4 controller
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

            dt_reg <= '0;

            Ixx_reg <= '0;
            Iyy_reg <= '0;
            Izz_reg <= '0;

            Ixx_inv_reg <= '0;
            Iyy_inv_reg <= '0;
            Izz_inv_reg <= '0;

            x2_x <= '0;
            x2_y <= '0;
            x2_z <= '0;

            x3_x <= '0;
            x3_y <= '0;
            x3_z <= '0;

            x4_x <= '0;
            x4_y <= '0;
            x4_z <= '0;

            k1_x <= '0;
            k1_y <= '0;
            k1_z <= '0;

            k2_x <= '0;
            k2_y <= '0;
            k2_z <= '0;

            k3_x <= '0;
            k3_y <= '0;
            k3_z <= '0;

            k4_x <= '0;
            k4_y <= '0;
            k4_z <= '0;

            dt_k_x_reg <= '0;
            dt_k_y_reg <= '0;
            dt_k_z_reg <= '0;

            weighted_x_reg <= '0;
            weighted_y_reg <= '0;
            weighted_z_reg <= '0;

            average_x_reg <= '0;
            average_y_reg <= '0;
            average_z_reg <= '0;

            delta_x_reg <= '0;
            delta_y_reg <= '0;
            delta_z_reg <= '0;

            div_x_dividend <= '0;
            div_y_dividend <= '0;
            div_z_dividend <= '0;

            div_x_quotient <= '0;
            div_y_quotient <= '0;
            div_z_quotient <= '0;

            div_x_remainder <= '0;
            div_y_remainder <= '0;
            div_z_remainder <= '0;

            div_x_negative <= 1'b0;
            div_y_negative <= 1'b0;
            div_z_negative <= 1'b0;

            div_count <= '0;

            wx_next <= '0;
            wy_next <= '0;
            wz_next <= '0;

        end else begin

            done <= 1'b0;

            case (state)

                // =================================================
                // Capture transaction
                // =================================================

                IDLE: begin

                    if (start) begin

                        wx_reg <= wx;
                        wy_reg <= wy;
                        wz_reg <= wz;

                        tx_reg <= torque_x;
                        ty_reg <= torque_y;
                        tz_reg <= torque_z;

                        dt_reg <= dt;

                        Ixx_reg <= Ixx;
                        Iyy_reg <= Iyy;
                        Izz_reg <= Izz;

                        Ixx_inv_reg <= Ixx_inv;
                        Iyy_inv_reg <= Iyy_inv;
                        Izz_inv_reg <= Izz_inv;

                        state <= START_K1;

                    end

                end


                // =================================================
                // K1 = f(w)
                // =================================================

                START_K1: begin
                    state <= WAIT_K1;
                end


                WAIT_K1: begin

                    if (dynamics_done) begin

                        k1_x <= dynamics_wx_dot;
                        k1_y <= dynamics_wy_dot;
                        k1_z <= dynamics_wz_dot;

                        state <= SCALE_X2_START;

                    end

                end


                // =================================================
                // dt*k1
                // =================================================

                SCALE_X2_START: begin
                    state <= SCALE_X2_WAIT;
                end


                SCALE_X2_WAIT: begin

                    if (rk_mul_x_done &&
                        rk_mul_y_done &&
                        rk_mul_z_done) begin

                        dt_k_x_reg <= rk_mul_x_result;
                        dt_k_y_reg <= rk_mul_y_result;
                        dt_k_z_reg <= rk_mul_z_result;

                        state <= CALC_X2;

                    end

                end


                // =================================================
                // x2 = w + (dt*k1)/2
                // =================================================

                CALC_X2: begin

                    x2_x <= wx_reg + (dt_k_x_reg >>> 1);
                    x2_y <= wy_reg + (dt_k_y_reg >>> 1);
                    x2_z <= wz_reg + (dt_k_z_reg >>> 1);

                    state <= START_K2;

                end


                // =================================================
                // K2 = f(x2)
                // =================================================

                START_K2: begin
                    state <= WAIT_K2;
                end


                WAIT_K2: begin

                    if (dynamics_done) begin

                        k2_x <= dynamics_wx_dot;
                        k2_y <= dynamics_wy_dot;
                        k2_z <= dynamics_wz_dot;

                        state <= SCALE_X3_START;

                    end

                end


                // =================================================
                // dt*k2
                // =================================================

                SCALE_X3_START: begin
                    state <= SCALE_X3_WAIT;
                end


                SCALE_X3_WAIT: begin

                    if (rk_mul_x_done &&
                        rk_mul_y_done &&
                        rk_mul_z_done) begin

                        dt_k_x_reg <= rk_mul_x_result;
                        dt_k_y_reg <= rk_mul_y_result;
                        dt_k_z_reg <= rk_mul_z_result;

                        state <= CALC_X3;

                    end

                end


                // =================================================
                // x3 = w + (dt*k2)/2
                // =================================================

                CALC_X3: begin

                    x3_x <= wx_reg + (dt_k_x_reg >>> 1);
                    x3_y <= wy_reg + (dt_k_y_reg >>> 1);
                    x3_z <= wz_reg + (dt_k_z_reg >>> 1);

                    state <= START_K3;

                end


                // =================================================
                // K3 = f(x3)
                // =================================================

                START_K3: begin
                    state <= WAIT_K3;
                end


                WAIT_K3: begin

                    if (dynamics_done) begin

                        k3_x <= dynamics_wx_dot;
                        k3_y <= dynamics_wy_dot;
                        k3_z <= dynamics_wz_dot;

                        state <= SCALE_X4_START;

                    end

                end


                // =================================================
                // dt*k3
                // =================================================

                SCALE_X4_START: begin
                    state <= SCALE_X4_WAIT;
                end


                SCALE_X4_WAIT: begin

                    if (rk_mul_x_done &&
                        rk_mul_y_done &&
                        rk_mul_z_done) begin

                        dt_k_x_reg <= rk_mul_x_result;
                        dt_k_y_reg <= rk_mul_y_result;
                        dt_k_z_reg <= rk_mul_z_result;

                        state <= CALC_X4;

                    end

                end


                // =================================================
                // x4 = w + dt*k3
                // =================================================

                CALC_X4: begin

                    x4_x <= wx_reg + dt_k_x_reg;
                    x4_y <= wy_reg + dt_k_y_reg;
                    x4_z <= wz_reg + dt_k_z_reg;

                    state <= START_K4;

                end


                // =================================================
                // K4 = f(x4)
                // =================================================

                START_K4: begin
                    state <= WAIT_K4;
                end


                WAIT_K4: begin

                    if (dynamics_done) begin

                        k4_x <= dynamics_wx_dot;
                        k4_y <= dynamics_wy_dot;
                        k4_z <= dynamics_wz_dot;

                        state <= FINAL_WEIGHT;

                    end

                end


                // =================================================
                // weighted = k1 + 2*k2 + 2*k3 + k4
                // =================================================

                FINAL_WEIGHT: begin

                    weighted_x_reg <=
                        $signed({{3{k1_x[31]}}, k1_x}) +
                        ($signed({{3{k2_x[31]}}, k2_x}) <<< 1) +
                        ($signed({{3{k3_x[31]}}, k3_x}) <<< 1) +
                        $signed({{3{k4_x[31]}}, k4_x});

                    weighted_y_reg <=
                        $signed({{3{k1_y[31]}}, k1_y}) +
                        ($signed({{3{k2_y[31]}}, k2_y}) <<< 1) +
                        ($signed({{3{k3_y[31]}}, k3_y}) <<< 1) +
                        $signed({{3{k4_y[31]}}, k4_y});

                    weighted_z_reg <=
                        $signed({{3{k1_z[31]}}, k1_z}) +
                        ($signed({{3{k2_z[31]}}, k2_z}) <<< 1) +
                        ($signed({{3{k3_z[31]}}, k3_z}) <<< 1) +
                        $signed({{3{k4_z[31]}}, k4_z});

                    state <= DIV_START;

                end


                // =================================================
                // Initialize exact divide-by-6
                // =================================================

                DIV_START: begin

                    div_x_negative <= weighted_x_reg[34];
                    div_y_negative <= weighted_y_reg[34];
                    div_z_negative <= weighted_z_reg[34];

                    if (weighted_x_reg[34])
                        div_x_dividend <=
                            (~$unsigned(weighted_x_reg)) + 35'd1;
                    else
                        div_x_dividend <=
                            $unsigned(weighted_x_reg);

                    if (weighted_y_reg[34])
                        div_y_dividend <=
                            (~$unsigned(weighted_y_reg)) + 35'd1;
                    else
                        div_y_dividend <=
                            $unsigned(weighted_y_reg);

                    if (weighted_z_reg[34])
                        div_z_dividend <=
                            (~$unsigned(weighted_z_reg)) + 35'd1;
                    else
                        div_z_dividend <=
                            $unsigned(weighted_z_reg);

                    div_x_quotient <= '0;
                    div_y_quotient <= '0;
                    div_z_quotient <= '0;

                    div_x_remainder <= '0;
                    div_y_remainder <= '0;
                    div_z_remainder <= '0;

                    div_count <= 6'd0;

                    state <= DIV_RUN;

                end


                // =================================================
                // Restoring divide by 6
                // =================================================

                DIV_RUN: begin

                    // X
                    div_x_dividend <=
                        {div_x_dividend[33:0], 1'b0};

                    div_x_quotient <=
                        {div_x_quotient[33:0],
                         (div_x_trial >= 4'd6)};

                    if (div_x_trial >= 4'd6)
                        div_x_remainder <=
                            div_x_trial - 4'd6;
                    else
                        div_x_remainder <=
                            div_x_trial[2:0];


                    // Y
                    div_y_dividend <=
                        {div_y_dividend[33:0], 1'b0};

                    div_y_quotient <=
                        {div_y_quotient[33:0],
                         (div_y_trial >= 4'd6)};

                    if (div_y_trial >= 4'd6)
                        div_y_remainder <=
                            div_y_trial - 4'd6;
                    else
                        div_y_remainder <=
                            div_y_trial[2:0];


                    // Z
                    div_z_dividend <=
                        {div_z_dividend[33:0], 1'b0};

                    div_z_quotient <=
                        {div_z_quotient[33:0],
                         (div_z_trial >= 4'd6)};

                    if (div_z_trial >= 4'd6)
                        div_z_remainder <=
                            div_z_trial - 4'd6;
                    else
                        div_z_remainder <=
                            div_z_trial[2:0];


                    if (div_count == 6'd34) begin

                        state <= DIV_FINISH;

                    end else begin

                        div_count <= div_count + 6'd1;

                    end

                end


                // =================================================
                // Restore divider signs
                // =================================================

                DIV_FINISH: begin

                    if (div_x_negative)
                        average_x_reg <=
                            -$signed(div_x_quotient[31:0]);
                    else
                        average_x_reg <=
                            $signed(div_x_quotient[31:0]);

                    if (div_y_negative)
                        average_y_reg <=
                            -$signed(div_y_quotient[31:0]);
                    else
                        average_y_reg <=
                            $signed(div_y_quotient[31:0]);

                    if (div_z_negative)
                        average_z_reg <=
                            -$signed(div_z_quotient[31:0]);
                    else
                        average_z_reg <=
                            $signed(div_z_quotient[31:0]);

                    state <= FINAL_SCALE_START;

                end


                // =================================================
                // delta = qmul(dt, average)
                // =================================================

                FINAL_SCALE_START: begin
                    state <= FINAL_SCALE_WAIT;
                end


                FINAL_SCALE_WAIT: begin

                    if (rk_mul_x_done &&
                        rk_mul_y_done &&
                        rk_mul_z_done) begin

                        delta_x_reg <= rk_mul_x_result;
                        delta_y_reg <= rk_mul_y_result;
                        delta_z_reg <= rk_mul_z_result;

                        state <= FINAL_UPDATE;

                    end

                end


                // =================================================
                // w_next = w + delta
                // =================================================

                FINAL_UPDATE: begin

                    wx_next <= wx_reg + delta_x_reg;
                    wy_next <= wy_reg + delta_y_reg;
                    wz_next <= wz_reg + delta_z_reg;

                    state <= DONE_STATE;

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
