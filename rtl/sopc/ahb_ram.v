//***************************************************************************
//   Copyright(c) 2017, Lyu Yang
//   All rights reserved
//
//   File name        :   ahb_ram.v
//   Module name      :
//   Author           :   Lyu Yang
//   Email            :
//   Date             :   2016-12-00
//   Version          :   v1.0
//
//   Abstract         :
//
//   Modification history
//   ------------------------------------------------------------------------
// Version       Date(yyyy/mm/dd)   name
// Description
//
// $Log$
//***************************************************************************
`timescale 1ns / 1ps
module ahb_ram (
    input   wire                hclk        ,
    input   wire                hreset_n    ,
    input   wire                hsel        ,
    input   wire    [1:0]       htrans      ,
    input   wire                hwrite      ,
    input   wire    [2:0]       hsize       ,
    input   wire    [31:0]      haddr       ,
    input   wire    [31:0]      hwdata      ,
    input   wire                hready_in   ,
    output  reg     [31:0]      hrdata      ,
    output                      hready_out  ,
    output  wire    [1:0]       hresp
);

localparam      MEM_SIZE = 4096;

wire            hbus_ena;
reg             hbus_ena_d;
reg     [3:0]   mem_wstrb;
reg     [31:0]  mem [0:MEM_SIZE-1];

// Memory Init
initial $readmemh("../frw/led_test.txt", mem);

// AHB response always OKAY
assign hresp = 2'h0;
assign hready_out = 1'b1;
assign hbus_ena = hsel & hready_in & htrans[1];

always @(posedge hclk, negedge hreset_n)
    if(~hreset_n)
        hbus_ena_d <= 1'b0;
    else
        hbus_ena_d <= hbus_ena;

always @(*)
    if(hbus_ena & hwrite) begin
        case(hsize)
            3'b000: begin
                case(haddr[1:0])
                    2'b00: mem_wstrb = 4'b0001;
                    2'b01: mem_wstrb = 4'b0010;
                    2'b10: mem_wstrb = 4'b0100;
                    2'b11: mem_wstrb = 4'b1000;
                endcase
            end
            3'b001: begin
                case(haddr[0])
                    1'b0: mem_wstrb = 4'b0011;
                    1'b1: mem_wstrb = 4'b1100;
                endcase
            end
            3'b010: begin
                mem_wstrb = 4'b1111;
            end
            default: begin
                mem_wstrb = 4'b0000;
            end
        endcase
    end
    else begin
        mem_wstrb = 4'b0000;
    end

always @(posedge hclk)
    if(hbus_ena & ~hbus_ena_d) begin
        if(mem_wstrb[0])
            mem[haddr[31:2]][ 7: 0] <= hwdata[ 7: 0];
        if(mem_wstrb[1])
            mem[haddr[31:2]][15: 8] <= hwdata[15: 8];
        if(mem_wstrb[2])
            mem[haddr[31:2]][23:16] <= hwdata[23:16];
        if(mem_wstrb[3])
            mem[haddr[31:2]][31:24] <= hwdata[31:24];
    end

always @(posedge hclk)
    if(hbus_ena)
        hrdata <= mem[haddr[31:2]];

endmodule

