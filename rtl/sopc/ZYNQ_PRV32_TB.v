//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:54:27 07/06/2019 
// Design Name: 
// Module Name:    ZYNQ_PRV32_TB 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ns
module ZYNQ_PRV32_TB;

reg             clk;
reg             rst_n;
wire    [3:0]   led;
wire            uart_tx;
wire            uart_rx;

ZYNQ_PRV32 U_DUT (
    .clk                (clk            ),
    .rst_n              (rst_n          ),
    .uart_txd           (uart_tx        ),
    .uart_rxd           (uart_rx        ),
    .led                (led            )
);

initial forever #10 clk = ~clk;

initial begin
    clk   = 1'b0;
    rst_n = 1'b0;
    #100;
    rst_n = 1'b1;
    #10000000;
    $finish;
end

initial begin
    $vcdplusfile("ZYNQ_PRV32_TB.vpd");
    $vcdpluson(0);
end

endmodule
