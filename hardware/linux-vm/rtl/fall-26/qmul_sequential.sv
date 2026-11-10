`timescale 1ns / 1ps

module qmul_sequential (
    input  logic clk,
    input  logic rst,

    input  logic start,
    output logic done,

    input  logic signed [31:0] a,
    input  logic signed [31:0] b,

    output logic signed [31:0] result
);

    // ============================================================
    // Exact sequential signed 32x32 multiplier
    //
    // Numerically equivalent to:
    //
    // logic signed [63:0] product;
    // product = a * b;
    // result  = product >>> 16;
    //
    // Multiplication is performed on unsigned magnitudes using
    // shift-and-add. The sign is restored to the complete 64-bit
    // product before the arithmetic Q16.16 shift.
    // ============================================================

    typedef enum logic [1:0] {
        IDLE,
        MULTIPLY,
        FINISH
    } state_t;

    state_t state;

    logic [63:0] multiplicand_reg;
    logic [31:0] multiplier_reg;
    logic [63:0] accumulator_reg;

    logic signed [63:0] product_reg;

    logic product_negative;

    logic [5:0] bit_count;

    logic [63:0] accumulator_next;


    // ============================================================
    // Current shift-and-add result
    // ============================================================

    always_comb begin

        if (multiplier_reg[0])
            accumulator_next =
                accumulator_reg + multiplicand_reg;
        else
            accumulator_next =
                accumulator_reg;

    end


    // ============================================================
    // Sequential multiplier
    // ============================================================

    always_ff @(posedge clk) begin

        if (rst) begin

            state <= IDLE;

            done <= 1'b0;
            result <= '0;

            multiplicand_reg <= '0;
            multiplier_reg <= '0;
            accumulator_reg <= '0;

            product_reg <= '0;
            product_negative <= 1'b0;

            bit_count <= '0;

        end else begin

            // done is a one-clock pulse.
            done <= 1'b0;

            case (state)

                // =================================================
                // Capture operands and convert them to magnitudes.
                // =================================================

                IDLE: begin

                    if (start) begin

                        product_negative <= a[31] ^ b[31];

                        if (a[31])
                            multiplicand_reg <=
                                {32'b0, ((~$unsigned(a)) + 32'd1)};
                        else
                            multiplicand_reg <=
                                {32'b0, $unsigned(a)};

                        if (b[31])
                            multiplier_reg <=
                                (~$unsigned(b)) + 32'd1;
                        else
                            multiplier_reg <=
                                $unsigned(b);

                        accumulator_reg <= 64'd0;
                        bit_count <= 6'd0;

                        state <= MULTIPLY;

                    end
                end


                // =================================================
                // One multiplier bit per clock.
                // =================================================

                MULTIPLY: begin

                    if (bit_count == 6'd31) begin

                        // accumulator_next includes the contribution
                        // of the final multiplier bit.
                        if (product_negative)
                            product_reg <=
                                -$signed(accumulator_next);
                        else
                            product_reg <=
                                $signed(accumulator_next);

                        state <= FINISH;

                    end else begin

                        accumulator_reg <= accumulator_next;

                        multiplicand_reg <=
                           multiplicand_reg << 1;

                        multiplier_reg <=
                            multiplier_reg >> 1;

                        bit_count <= bit_count + 6'd1;

                    end

                end


                // =================================================
                // Restore Q16.16 representation.
                //
                // Arithmetic shift occurs on the complete signed
                // 64-bit product exactly as in the old qmul().
                // =================================================

                FINISH: begin

                    result <= product_reg >>> 16;
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
