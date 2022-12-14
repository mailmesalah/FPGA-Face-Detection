/*

	Face Detection Top Module
	Started on 07/04/13

*/

module FaceDetection
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_24,						//	24 MHz
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,								//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[9:0]
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,							//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	VGA		////////////////////////////
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_R,   						//	VGA Red[3:0]
		VGA_G,	 						//	VGA Green[3:0]
		VGA_B,  							//	VGA Blue[3:0]
		////////////////////	GPIO	////////////////////////////								
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input	[1:0]	CLOCK_24;				//	24 MHz
input	[1:0]	CLOCK_27;				//	27 MHz
input			CLOCK_50;				//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;						//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[9:0]	SW;						//	Toggle Switch[9:0]
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;			//	SDRAM Data bus 16 Bits
output	[11:0]	DRAM_ADDR;			//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;			//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;			//	SDRAM High-byte Data Mask
output			DRAM_WE_N;			//	SDRAM Write Enable
output			DRAM_CAS_N;			//	SDRAM Column Address Strobe
output			DRAM_RAS_N;			//	SDRAM Row Address Strobe
output			DRAM_CS_N;			//	SDRAM Chip Select
output			DRAM_BA_0;			//	SDRAM Bank Address 0
output			DRAM_BA_1;			//	SDRAM Bank Address 0
output			DRAM_CLK;			//	SDRAM Clock
output			DRAM_CKE;			//	SDRAM Clock Enable
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;			//	I2C Data
output			I2C_SCLK;			//	I2C Clock
////////////////////////	VGA			////////////////////////////
output			VGA_HS;				//	VGA H_SYNC
output			VGA_VS;				//	VGA V_SYNC
output	[3:0]	VGA_R;   			//	VGA Red[3:0]
output	[3:0]	VGA_G;	 			//	VGA Green[3:0]
output	[3:0]	VGA_B;   			//	VGA Blue[3:0]

////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_1;				//	GPIO Connection 1
///////////////////////////////////////////////////////////////////
//=============================================================================
// REG/WIRE declarations
//=============================================================================

//	CCD
wire	[11:0]	CCD_DATA;
wire			CCD_SDAT;
wire			CCD_SCLK;
wire			CCD_FLASH;
wire			CCD_FVAL;
wire			CCD_LVAL;
wire			CCD_PIXCLK;
wire			CCD_MCLK;				//	CCD Master Clock

wire	[15:0]	Read_DATA1;
wire	[15:0]	Read_DATA2;
wire			VGA_CTRL_CLK;
wire	[11:0]	mCCD_DATA;
wire			mCCD_DVAL;
wire			mCCD_DVAL_d;
wire	[15:0]	X_Cont;
wire	[15:0]	Y_Cont;
wire	[9:0]	X_ADDR;
wire	[31:0]	Frame_Cont;
wire			DLY_RST_0;
wire			DLY_RST_1;
wire			DLY_RST_2;
wire			Read;
reg		[11:0]	rCCD_DATA;
reg				rCCD_LVAL;
reg				rCCD_FVAL;
wire	[11:0]	sCCD_R;
wire	[11:0]	sCCD_G;
wire	[11:0]	sCCD_B;
wire			sCCD_DVAL;
wire	[3:0]	VGA_R;   				//	VGA Red[9:0]
wire	[3:0]	VGA_G;	 				//	VGA Green[9:0]
wire	[3:0]	VGA_B;   				//	VGA Blue[9:0]
reg	[1:0]		rClk;
wire			sdram_ctrl_clk;

//=============================================================================
// Structural coding
//=============================================================================
assign	CCD_DATA[0]	=	GPIO_1[13];
assign	CCD_DATA[1]	=	GPIO_1[12];
assign	CCD_DATA[2]	=	GPIO_1[11];
assign	CCD_DATA[3]	=	GPIO_1[10];
assign	CCD_DATA[4]	=	GPIO_1[9];
assign	CCD_DATA[5]	=	GPIO_1[8];
assign	CCD_DATA[6]	=	GPIO_1[7];
assign	CCD_DATA[7]	=	GPIO_1[6];
assign	CCD_DATA[8]	=	GPIO_1[5];
assign	CCD_DATA[9]	=	GPIO_1[4];
assign	CCD_DATA[10]=	GPIO_1[3];
assign	CCD_DATA[11]=	GPIO_1[1];
assign	GPIO_1[16]	=	CCD_MCLK;
assign	CCD_FVAL	=	GPIO_1[22];
assign	CCD_LVAL	=	GPIO_1[21];
assign	CCD_PIXCLK	=	GPIO_1[0];
assign	GPIO_1[19]	=	1'b1;  // TRIGGER
assign	GPIO_1[17]	=	DLY_RST_1;

assign	VGA_CTRL_CLK=	rClk[0];
assign	VGA_CLK		=	~rClk[0];

always@(posedge CLOCK_50)	rClk	<=	rClk+1;

wire	[9:0]	oVGA_R;
wire	[9:0]	oVGA_G;
wire	[9:0]	oVGA_B;

assign	VGA_R		=	oVGA_R[9:6];
assign	VGA_G		=	oVGA_G[9:6];
assign	VGA_B		=	oVGA_B[9:6];

/*
// Skin Detection
assign	VGA_R		=	oVGA_R[9:6]?0:oVGA_R[9:6];
assign	VGA_G		=	oVGA_G[9:6]?0:oVGA_G[9:6];
assign	VGA_B		=	oVGA_B[9:6]?0:oVGA_B[9:6];
*/

always@(posedge CCD_PIXCLK)
begin
	rCCD_DATA	<=	CCD_DATA;
	rCCD_LVAL	<=	CCD_LVAL;
	rCCD_FVAL	<=	CCD_FVAL;
end

CCD_Capture			u3	(	
							.oDATA      		(mCCD_DATA),
							.oDVAL      		(mCCD_DVAL),
							.oX_Cont    		(X_Cont),
							.oY_Cont    		(Y_Cont),
							.oFrame_Cont		(Frame_Cont),
							.iDATA      		(rCCD_DATA),
							.iFVAL      		(rCCD_FVAL),
							.iLVAL      		(rCCD_LVAL),
							.iSTART     		(!KEY[3]),							
							.iCLK       		(CCD_PIXCLK),
							.iRST       		(DLY_RST_2)
						);

RAW2RGB				u4	(	
							.iCLK   			(CCD_PIXCLK),
							.iRST   			(DLY_RST_1),
							.iDATA  			(mCCD_DATA),
							.iDVAL  			(mCCD_DVAL),
							.oRed   			(sCCD_R),
							.oGreen 			(sCCD_G),
							.oBlue  			(sCCD_B),
							.oDVAL  			(sCCD_DVAL),
							.iX_Cont			(X_Cont),
							.iY_Cont			(Y_Cont)
						);

Sdram_Control_4Port	u7	(	//	HOST Side						
							.REF_CLK			(CLOCK_50),
							.RESET_N			(1'b1),
							.CLK   				(sdram_ctrl_clk),

									//	FIFO Write Side 1
							.WR1_DATA			({1'b0,sCCD_G[11:7],sCCD_B[11:2]}),
							.WR1    			(sCCD_DVAL),
							.WR1_ADDR			(0),
							.WR1_MAX_ADDR		(640*480),
							.WR1_LENGTH 		(9'h100),
							.WR1_LOAD	  		(!DLY_RST_0),
							.WR1_CLK	  		(~CCD_PIXCLK),

								  //	FIFO Write Side 2
							.WR2_DATA    		({1'b0,sCCD_G[6:2],sCCD_R[11:2]}),
							.WR2			  	(sCCD_DVAL),
							.WR2_ADDR    		(22'h100000),
							.WR2_MAX_ADDR		(22'h100000+640*480),
							.WR2_LENGTH  		(9'h100),
							.WR2_LOAD    		(!DLY_RST_0),
							.WR2_CLK     		(~CCD_PIXCLK),


								  //	FIFO Read Side 1
							.RD1_DATA	 		(Read_DATA1),
							.RD1		 		(Read),
				        	.RD1_ADDR    		(0),
							.RD1_MAX_ADDR		(640*480),
							.RD1_LENGTH  		(9'h100),
							.RD1_LOAD	 		(!DLY_RST_0),
							.RD1_CLK	 		(~VGA_CTRL_CLK),
							
								  //	FIFO Read Side 2
							.RD2_DATA	 		(Read_DATA2),
							.RD2         		(Read),
							.RD2_ADDR    		(22'h100000),
							.RD2_MAX_ADDR		(22'h100000+640*480),
							.RD2_LENGTH  		(9'h100),
							.RD2_LOAD    		(!DLY_RST_0),
							.RD2_CLK     		(~VGA_CTRL_CLK),
							
								  //	SDRAM Side
							.SA			  		(DRAM_ADDR),
							.BA			  		({DRAM_BA_1,DRAM_BA_0}),
							.CS_N		  		(DRAM_CS_N),
        					.CKE		  		(DRAM_CKE),
							.RAS_N		  		(DRAM_RAS_N),
							.CAS_N		  		(DRAM_CAS_N),
							.WE_N		  		(DRAM_WE_N),
							.DQ			  		(DRAM_DQ),
							.DQM		  		({DRAM_UDQM,DRAM_LDQM})
					);


VGA_Controller		u1	(	//	Host Side
							.oRequest   		(Read),
							.iRed       		(Read_DATA2[9:0]),
							.iGreen     		({Read_DATA1[14:10],Read_DATA2[14:10]}),
							.iBlue      		(Read_DATA1[9:0]),
							//.iRed       (sCCD_R[11:2]),
							//.iGreen     (sCCD_G[11:2]),
							//.iBlue      (sCCD_B[11:2]),
							//	VGA Side
							.oVGA_R     		(oVGA_R),
							.oVGA_G     		(oVGA_G),
							.oVGA_B     		(oVGA_B),
							.oVGA_H_SYNC		(VGA_HS),
							.oVGA_V_SYNC		(VGA_VS),
							//	Control Signal
							.iCLK       		(VGA_CTRL_CLK),
							.iRST_N     		(DLY_RST_2)
						);


Reset_Delay			u2	(	
							.iCLK  				(CLOCK_50),
							.iRST  				(KEY[0]),
							.oRST_0				(DLY_RST_0),
							.oRST_1				(DLY_RST_1),
							.oRST_2				(DLY_RST_2)
						);


sdram_pll 			u6	(
							.inclk0				(CLOCK_50),
							.c0    				(sdram_ctrl_clk),
							.c1    				(DRAM_CLK)
						);

assign CCD_MCLK = rClk[0];



I2C_CCD_Config 		u8	(	//	Host Side
							.iCLK			  	(CLOCK_50),
							.iRST_N         	(DLY_RST_2),
							.iZOOM_MODE_SW  	(SW[8]),
							.iEXPOSURE_ADJ  	(KEY[1]),
							.iEXPOSURE_DEC_p	(SW[0]),
							 //	I2C Side
							.I2C_SCLK		  	(GPIO_1[24]),
							.I2C_SDAT		  	(GPIO_1[23])
						);

endmodule