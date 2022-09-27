/*************************************************************************/
/* File:        bayer2rgb.v
 *
 * Description: Translate bayer pattern to RGB
 *    Bayer pattern:
 *    G B G B
 *    R G R G
 *    G B G B
 *    R G R G
 **************************************************************************/
`timescale 1ns/1ns

// Convert Bayer pattern to RGB
module bayer2rgb
  (input           clk,
   input           reset,
   input    [10:0] colAddrIn,    // column count
   input    [10:0] lineAddrIn,    // row count
   input     [7:0] pixelIn,
   input           pixelValidIn,   // pixelValidIn
   output    [7:0] red,
   output    [7:0] green,
   output    [7:0] blue,
   output reg [8:0] colAddrOut,
   output reg [7:0] lineAddrOut,
   output reg      pixelValidOut);   // pixelValidOut (on 1,1 pixel - divide by 4)

   localparam NUMCOLS = 640;

   // Memory
   reg [7:0] 	   lineup [0:NUMCOLS-1];
   // 2x2 window of pixels
   reg [7:0] 	   prevPixelup, thisPixelup;
   reg [7:0] 	   prevPixel, thisPixel;
   // Pipeline registers for inputs
   reg    [10:0] colAddr;   
   reg    [10:0] lineAddr;  
   reg     [7:0] pixel;
   reg           pixelValid;

   // Compute output values
   assign red = prevPixel;
   assign green = (prevPixelup + thisPixel + 1) >> 1;
   assign blue = thisPixelup;

  // Capture input data
  always @(posedge clk) begin
     // Pipeline registers
     colAddr <= colAddrIn;
     lineAddr <= lineAddrIn;
     pixelValid <= pixelValidIn;
     pixel <= pixelIn;
     pixelValidOut <= pixelValid && colAddr[0] && lineAddr[0];
     colAddrOut <= colAddr[9:1];
     lineAddrOut <= lineAddr[8:1];
    if (pixelValid) begin
       lineup[colAddr] <= pixel;
       prevPixel <= thisPixel;
       thisPixel <= pixel;
       prevPixelup <= thisPixelup;
       thisPixelup <= lineup[colAddr];
    end
  end

endmodule
