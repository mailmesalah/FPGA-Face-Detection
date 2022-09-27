/*

	Face Detection Top Module
	Started on 07/04/13

*/

module FaceDetection
	(
		////////////////////	Clock Input	 	////////////////////	 
		input Clock24, input Clock27, input Clock50, input ExtClock,
		////////////////////	Push Button		////////////////////
		input 		Reset,								
		////////////////////	VGA		////////////////////////////
		output VGAhSync, output VGAvSync, 
		output [3:0]VGAr, output [3:0]VGAg,	output [3:0]VGAb,
		////////////////////	GPIO	////////////////////////////
		input [11:0]CamData, input CamLVal,	input CamFVal, input CamPixClock, 
		output CamReset, output CamExtClock,
		////////////////////	SDRAM	////////////////////////////
		inout [15:0]DRAM_DQ, output [11:0]DRAM_ADDR, 
		output DRAM_LDQM, output DRAM_UDQM, output DRAM_WE_N, 
		output DRAM_CAS_N, output DRAM_RAS_N, output DRAM_CS_N,
		output DRAM_BA_0, output DRAM_BA_1, output DRAM_CLK,
		output DRAM_CKE
	);

///////////////////////////////////////////////////////////////////
//=============================================================================
// REG/WIRE declarations
//=============================================================================
wire 			wCamClock;
wire 			wCamLVal;
wire 			wCamFVal;
wire	[11:0]	wCamData;

wire	[11:0]	rCamData;
wire 			rCamDVal;
wire 	[1:0]	rCamState;

wire 			rReady;
wire	[11:0]	rVGARed;
wire	[11:0]	rVGAGreen;
wire	[11:0]	rVGABlue;

wire SDRAMClock;

//=============================================================================
// Structural coding
//=============================================================================
assign	wCamClock	=	CamPixClock;
assign	wCamData	=	CamData;
assign	wCamLVal	=	CamLVal;
assign	wCamFVal	=	CamFVal;

assign	CamExtClock=	Clock50;

CaptureCamera CCDCamera(	
		.oData(rCamData),.oDVal(rCamDVal),.oState(rCamState),
		.iData(wCamData),.iFVal(wCamFVal),.iLVal(wCamLVal),
		.iClock(wCamClock),.iReset(Reset)					
		);
ImageProcessor(
		.iClock(SDRAMClock),.iReset(Reset),
		//Camera Input
		.iCameraState(rCamState),.iCameraData(rCamData),.iCameraDVal(rCamDVal),
		.iCameraClock(wCamClock),
		//VGA Output
		.oVGARed(rVGARed),.oVGAGreen(rVGAGreen),.oVGABlue(rVGABlue),
		.iVGAClock(Clock27),.oReady(rReady),
		//	SDRAM
		.Address(DRAM_ADDR),.BankAddress({DRAM_BA_1,DRAM_BA_0}),.ChipEnable(DRAM_CS_N),
		.ClockEnable(DRAM_CKE),.RowAddressStrobe(DRAM_RAS_N),.ColumnAddressStrobe(DRAM_CAS_N),
		.WriteEnable(DRAM_WE_N),.DataMask({DRAM_UDQM,DRAM_LDQM}),    
		.Data(DRAM_DQ)     				
		);

VGAController VGA (
		.iClock(Clock27),.iReset(Reset),.iReady(rReady),
		.iRed(rVGARed[11:2]),.iGreen(rVGAGreen[11:2]),.iBlue(rVGABlue[11:2]),
		.oRed(VGAr),.oGreen(VGAg),.oBlue(VGAb),
		.oHSync(VGAhSync),.oVSync(VGAvSync)		
		);
		
		
ClockDivider CD(
		.clkswitch(1'b0),
		.inclk0(Clock50),
		.inclk1(),
		.c0(SDRAMClock),
		.c1()
		);

endmodule

		/*
		
		Image processeor
				
		*/
module ImageProcessor(
				input iClock, input iReset,
				//Camera Input
				input [1:0]iCameraState, input [11:0]iCameraData, input iCameraDVal,
				input iCameraClock,
				//VGA Output
				output [11:0]oVGARed, output [11:0]oVGAGreen, output [11:0]oVGABlue,
				input iVGAClock,output oReady,
				//	SDRAM
				output  [11:0]  Address, output  [1:0] BankAddress, output  [1:0] ChipEnable,
				output  ClockEnable, output  RowAddressStrobe, output  ColumnAddressStrobe,
				output  WriteEnable, output  [1:0]  DataMask,    
				inout   [15:0] Data     				
					);
	
	wire		rAck;
	wire		rColOrRow;
					
	reg [2:0]State; 
	reg 	rReadWrite;		
		
	reg [11:0]WRowAddress;
	reg [7:0]WColAddress;
	
	reg [11:0]RRowAddress;
	reg [7:0]RColAddress;
	
	reg 	readCompleted;
	
	
	assign Address	=	(rReadWrite==1'b0) ? ((rColOrRow==1'b0) ? {4'b0,RColAddress} : RRowAddress) : ((rColOrRow==1'b0) ? {4'b0,WColAddress} : WRowAddress); 						
	assign BankAddress=2'b00;
	assign ClockEnable=1'b1;
	
	assign Data = (State==3'b001)?{4'b0,iCameraData}:Data;
	
	assign oVGARed=(State==3'b010)?Data[11:0]:0;
	assign oVGAGreen=(State==3'b010)?Data[11:0]:0;
	assign oVGABlue=(State==3'b010)?Data[11:0]:0;
	
	assign oReady= (State==3'b010)?1'b1:1'b0;
	
	ReadWritePixel	RWPixel(
		.RowAddressStrobe(RowAddressStrobe),.ColumnAddressStrobe(ColumnAddressStrobe),.WriteEnable(WriteEnable),
		.ChipEnable(ChipEnable),
		.Acknowledge(rAck),
		
		.ColOrRow(rColOrRow),
		.iClock (iClock) , .iReset(iReset),
		.WriteClock(iCameraClock),.ReadClock(iVGAClock), .ReadWrite(rReadWrite)					
				);
	// Calculating the Address for Writing Frame
	always@(posedge iCameraClock or negedge iReset)
	begin
		if(!iReset)
		begin
			WRowAddress=12'b0;
			WColAddress=8'b0;
		end
		
		else if(State==3'b001)
		begin
			/*
			if(WColAddress==255 && WRowAddress==1199)
			begin
				State=3'b000;//Reset
			end
			*/
			if((WColAddress+1) % 256 == 0)
			begin
				WRowAddress=WRowAddress+1;
				WColAddress=0;
			end 
			else
			begin
				WColAddress=WColAddress+1;
			end
		end
		
		else
		begin
			WRowAddress=12'b0;
			WColAddress=8'b0;
		end	
	end
	
	// Calculating the Address for Reading Frame
	always@(posedge iVGAClock or negedge iReset)
	begin
		if(!iReset)
		begin
			RRowAddress=12'b0;
			RColAddress=8'b0;
			
			readCompleted=1'b0;
		end 
		
		else if(State==3'b010)
		begin
			if(RColAddress==255 && RRowAddress==1199)
			begin
				readCompleted=1'b1;//Reset
			end
			else if((RColAddress+1) % 256 == 0)
			begin
				RRowAddress=RRowAddress+1;
				RColAddress=0;
			end 
			else
			begin
				RColAddress=RColAddress+1;
			end
		end
			
		else
		begin
			RRowAddress=12'b0;
			RColAddress=8'b0;
			
			readCompleted=1'b0;
		end
	end
	
	//States for Writting Frame and Reading Frame
	always@(posedge iClock or negedge iReset)
	begin
		if(!iReset)
		begin			
			State=3'b0;// Halt State				
		end
		
		else
		begin
			if(State==3'b000 && iCameraState==2'b01)// Check for Start Point of a Frame
			begin
				State=3'b001; //Write Frame To SDRam State
				rReadWrite=1'b1;					
			end
			
			else if(State==3'b001 && iCameraState==2'b11)//End of Frame?
			begin
				State=3'b010; //Read Frame From SDRam State To VGA
				rReadWrite=1'b0;				
			end
			
			else if(State==3'b010 && readCompleted==1'b1)
			begin
				State=3'b000; //End Of Reading So Resetting
			end
		end
	end

endmodule

module ReadWritePixel(
				output  RowAddressStrobe, output  ColumnAddressStrobe, output  WriteEnable,
				output  [1:0] ChipEnable,
				output Acknowledge,
				output ColOrRow,
				input iClock /* High Frequency Clock */ , input iReset,
				input WriteClock/* Camera Clock */,input ReadClock/* VGA Clock */, input [1:0]ReadWrite					
					);
					
	reg [3:0]State;
	reg RAS,CAS,WE,CS,AKN;	
	reg rDataAvailable;
	reg rColOrRow;
	reg wColOrRow;
	
	assign	ColOrRow	=	rColOrRow; 						
	
	
	always@(posedge iClock or negedge iReset)
	begin
		if(!iReset)
		begin
			State=4'b0000;// NOP State
			
			CS=1'b1;			
		end
		
		else
		begin
			/************************
			Initialisation
			************************/
			if(State==4'b0000 && (ReadWrite==2'01 || ReadWrite==2'10))// if a new Data is Available
			begin
				
				CS=1'b1;
				
				State=4'b0001;		// 0001 Next State NOP											
			end		

			else if(State==4'b0001)	// 0001 Checks State NOP
			begin
				CS=1'b1;
				
				State=4'b0010;		// 0010 Next State Active				
				rColOrRow=1'b1;
			end
			
			/************************
			Active Command
			************************/
			else if(State==4'b0010)	// 0010 Checks State Active
			begin
				// For Active
				CS=1'b0;
				RAS=1'b1;
				CAS=1'b1;
				WE=1'b1;				
			
				State=4'b0011;		// 0011 Next State Nop				
			end
			
			else if(State==4'b0011)	// 0011 Checks State Nop
			begin
				CS=1'b1;
				
				if(ReadWrite==1'b0)	// Perform Read Operation
				begin
					State=4'b0100;	// 0100 Next State Read	
					rColOrRow=1'b0;			
				end
				
				else				// Perform Write Operation
				begin
					State=4'b1000;	// 1000 Next State Write
					rColOrRow=1'b0;
				end
			end
			/*******************************************************/
			
			/************************
			Read Command
			************************/
			else if(State==4'b0100)	// 0100 Checks State Read
			begin
				CS=1'b0;
				RAS=1'b0;
				CAS=1'b1;
				WE=1'b1;
				
				State=4'b0101;		// 0101 Next State Nop				
			end
			
			else if(State==4'b0101)	// 0101 Checks State Nop
			begin
				CS=1'b1;
				
				State=4'b0110;		// 0110 Next State Nop				
			end
			
			else if(State==4'b0110)	// 0110 Checks State Nop
			begin
				CS=1'b1;
				
				State=4'b0111;		// 0111 Next State Nop				
			end
			
			else if(State==4'b0111)	// 0111 Checks State Nop
			begin
				CS=1'b1;
				
				State=4'b0000;		// 0000 Next State Nop (Initial Waiting for Input to come)				
			end
			/*******************************************************/
			
			/************************
			Write Command
			************************/
			else if(State==4'b1000)	// 1000 Checks State Write
			begin
				CS=1'b0;
				RAS=1'b1;
				CAS=1'b0;
				WE=1'b0;
				
				State=4'b1001;		// 1001 Next State Nop				
			end
			
			else if(State==4'b1001)	// 1001 Checks State Nop
			begin
				CS=1'b1;
				
				State=4'b1010;		// 1010 Next State Nop				
			end
			
			else if(State==4'b1010)	// 1010 Checks State Nop
			begin
				CS=1'b1;
				
				State=4'b1011;		// 1011 Next State Nop				
			end
			
			else if(State==4'b1011)	// 1011 Checks State Nop
			begin
				CS=1'b1;
				
				State=4'b0000;		// 0000 Next State Nop (Initial Waiting for Input to come)				
			end
			/*******************************************************/
		end
	end			
					
endmodule

module CaptureCamera(
				output [11:0]oData, output oDVal, output [1:0]oState,
				input [11:0]iData, input iFVal, input iLVal,
				input iClock, input iReset					
					);

reg [15:0]	XCount,YCount;
reg [11:0]	rData;
reg [2:0]	rState;
reg 		rDVal;

reg			PrevFVal;
reg			PrevLVal;
reg			CurrFVal;
reg			CurrLVal;

parameter COLUMN_WIDTH = 1280;

assign	oData		=	rData;
assign	oDVal		=	rDVal;
assign 	oState		=	rState;

always@(posedge iClock or negedge iReset)
begin
	if(!iReset)
	begin
		rState	=2'b00;//Halt State
		XCount	=0;
		YCount	=0;
		rDVal	=1'b0;

		PrevFVal=1'b0;
		PrevLVal=1'b0;

	end
	else
	begin
		/*
		 * Calculating the X and Y Cordinates of Current Pixel and arranging for a Frame
		 *
		 */
		CurrFVal=iFVal;
		CurrLVal=iLVal;
		
		if(PrevFVal==1'b0 && PrevLVal==1'b0 && CurrFVal==1'b1 && CurrLVal==1'b1)//First Pixel of the Frame
		begin
			rState=2'b01;//Start State
			
			XCount=0;
			YCount=0;
			rDVal=1'b1;
			
			PrevFVal=1'b1;
			PrevLVal=1'b1;
		end
		else if(PrevFVal==1'b1 && PrevLVal==1'b1 && CurrFVal==1'b1 && CurrLVal==1'b0)//Reached End Of Line
		begin
			rState=2'b10;//Running State
			
			XCount=0;
			YCount=YCount+1;
			
			rDVal=1'b0;
			
			PrevFVal=1'b1;
			PrevLVal=1'b0;
		end
		else if(PrevFVal==1'b1 && PrevLVal==1'b0 && CurrFVal==1'b1 && CurrLVal==1'b1)//First pixel of a (next)Line in a Frame 
		begin
			rState=2'b10;//Running State
			
			rDVal=1'b1;
			
			PrevFVal=1'b1;
			PrevLVal=1'b1;
		end
		else if(PrevFVal==1'b1 && PrevLVal==1'b1 && CurrFVal==1'b0 && CurrLVal==1'b0)//Reached End Of Frame
		begin
			rState=2'b11;// Stop State
			
			XCount=0;
			YCount=0;
			rDVal=1'b0;
			
			PrevFVal=1'b0;
			PrevLVal=1'b0;
		end
		else if(PrevFVal==1'b1 && PrevLVal==1'b1 && CurrFVal==1'b1 && CurrLVal==1'b1)//Inside Frame next pixel in a line
		begin
			rState=2'b10;//Running State
			
			XCount=XCount+1;			
			
			rDVal=1'b1;
		end
		/******************************************************************************/
		
		/*
		 * Reducing Size of Camera Frame from to 640 X 480
		 * By only taking those pixels whose x and y coordinates are even.
		 */
		if({YCount[0],XCount[0]}==2'b00)
		begin
			rData=iData;
		end		
		else
		begin
			rDVal=1'b0;
			rData=12'b0;
		end
		/******************************************************************************/
	end	
end					
endmodule

/*
	VGA Controller
*/
module VGAController
		(
		input iClock,input iReset,input iReady, 
		input [9:0]iRed,input [9:0]iGreen,input [9:0]iBlue,
		output [3:0]oRed,output [3:0]oGreen,output [3:0]oBlue,
		output oHSync,output oVSync		
		);
		
//	Horizontal Parameter	( Pixel )
parameter	H_SYNC_CYC	=	96;
parameter	H_SYNC_BACK	=	48;
parameter	H_SYNC_ACT	=	640;	
parameter	H_SYNC_FRONT=	16;
parameter	H_SYNC_TOTAL=	800;

//	Vertical Parameter		( Line )
parameter	V_SYNC_CYC	=	2;
parameter	V_SYNC_BACK	=	33;
parameter	V_SYNC_ACT	=	480;	
parameter	V_SYNC_FRONT=	10;
parameter	V_SYNC_TOTAL=	525;

//	Internal Registers and Wires
reg		[11:0]		HCount;
reg		[11:0]		VCount;

reg				woHSync;
reg				woVSync;

//	Start Offset
parameter	X_START		=	H_SYNC_CYC+H_SYNC_BACK;
parameter	Y_START		=	V_SYNC_CYC+V_SYNC_BACK;

assign	oRed	=	(	HCount>=X_START 	&& HCount<X_START+H_SYNC_ACT &&
						VCount>=Y_START 	&& VCount<Y_START+V_SYNC_ACT )
						?	iRed[9:6]	:	0;
assign	oGreen	=	(	HCount>=X_START 	&& HCount<X_START+H_SYNC_ACT &&
						VCount>=Y_START 	&& VCount<Y_START+V_SYNC_ACT )
						?	iGreen[9:6]	:	0;
assign	oBlue	=	(	HCount>=X_START 	&& HCount<X_START+H_SYNC_ACT &&
						VCount>=Y_START 	&& VCount<Y_START+V_SYNC_ACT )
						?	iBlue[9:6]	:	0;		
		
assign oHSync=woHSync;
assign oVSync=woVSync;

//	H_Sync Generator & V_Sync Generator , Ref. 25.175 MHz Clock
always@(posedge iClock or negedge iReset)
begin
	if(!iReset || iReady)
	begin
		HCount		<=	0;
		woHSync		<=	0;
		
		VCount		<=	0;
		woVSync 	<=	0;
	end
	else
	begin
		//	H_Sync Counter
		if( HCount < H_SYNC_TOTAL )
			HCount	<=	HCount+1;
		else
			HCount	<=	0;
		
		//	H_Sync Generator
		if( HCount < H_SYNC_CYC )
			woHSync	<=	0;
		else
			woHSync	<=	1;
		
		//	When H_Sync Re-start
		if(HCount==0)
		begin
			//	V_Sync Counter
			if( VCount < V_SYNC_TOTAL )
				VCount	<=	VCount+1;
			else
				VCount	<=	0;
		
			//	V_Sync Generator
			if(	VCount < V_SYNC_CYC )
				woVSync	<=	0;
			else
				woVSync	<=	1;
		end
	end
end

endmodule
