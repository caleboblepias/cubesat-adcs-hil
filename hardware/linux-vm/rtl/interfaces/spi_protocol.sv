module spi_protocol (
        input  logic       sclk,
        input  logic       cs,
        input  logic       mosi,
        input  logic       rst,

        output logic       miso
);

        logic [7:0]        received_data;
        logic              done;
        logic [7:0]        tx_data;

        spi_slave_sclk spi_phy (
                .sclk(sclk),
                .cs(cs),
                .mosi(mosi),
                .tx_data(tx_data),
                .miso(miso),
                .done(done),
                .received_data(received_data)
        );

        typedef enum logic [1:0] {
                WAIT_CMD,
                WAIT_ADDR,
                WAIT_DATA
        } state_t;

        state_t            state;

        logic [7:0]        cmd;
        logic [7:0]        addr;
        logic [7:0]        regs [0:255];

        always_ff @(posedge sclk or posedge rst or posedge cs) begin
                if (rst || cs) begin
                        state   <= WAIT_CMD;
                        cmd     <= 8'd0;
                        addr    <= 8'd0;
                        tx_data <= 8'd0;

                end else begin

                        if (done) begin
                                case (state)

                                        WAIT_CMD: begin
                                                cmd   <= received_data;
                                                state <= WAIT_ADDR;
                                        end

                                        WAIT_ADDR: begin
                                                addr <= received_data;

                                                if (cmd == 8'h01) begin
                                                        state <= WAIT_DATA;

                                                end else if (cmd == 8'h02) begin
                                                        tx_data <= regs[received_data];
                                                        state   <= WAIT_CMD;
                                                end
                                        end

                                        WAIT_DATA: begin
                                                regs[addr] <= received_data;
                                                state      <= WAIT_CMD;
                                        end

                                endcase
                        end
                end
        end

endmodule
